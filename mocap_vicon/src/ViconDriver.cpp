#include <mocap_vicon/ViconDriver.h>

using namespace std;
using namespace Eigen;
namespace ViconSDK = ViconDataStreamSDK::CPP;

namespace mocap
{
    bool ViconDriver::init(rclcpp::Node& node)
    {
        // Read all parameters from the supplied node (the outer ViconNode), not
        // from *this ("vicon_driver"), so that values set via the launch file or
        // CLI are honoured.  ViconDriver inherits from MoCapDriverBase which is
        // itself a separate rclcpp::Node ("vicon_driver"), so ROS 2 parameter
        // scoping would otherwise hide any launch-file overrides.
        node.declare_parameter("server_address", string("192.154.4.124"));
        node.get_parameter("server_address", server_address);

        node.declare_parameter("model_list", vector<string>(0));
        node.get_parameter("model_list", model_list);

        node.declare_parameter("frame_rate", 100);
        node.get_parameter("frame_rate", frame_rate);

        node.declare_parameter("max_accel", 10.0);
        node.get_parameter("max_accel", max_accel);

        node.declare_parameter("publish_tf", false);
        node.get_parameter("publish_tf", publish_tf);

        node.declare_parameter("fixed_frame_id", string("mocap"));
        node.get_parameter("fixed_frame_id", fixed_frame_id);
        
        frame_interval = 1.0 / static_cast<double>(frame_rate);
        double& dt = frame_interval;

        process_noise.topLeftCorner<6, 6>() = 0.5*Matrix<double, 6, 6>::Identity()*dt*dt*max_accel;
        process_noise.bottomRightCorner<6, 6>() = Matrix<double, 6, 6>::Identity()*dt*max_accel;
        process_noise *= process_noise; // Make it a covariance

        measurement_noise = Matrix<double, 6, 6>::Identity()*1e-3;
        measurement_noise *= measurement_noise; // Make it a covariance

        model_set.insert(model_list.begin(), model_list.end());

        timespec ts_sleep;
        ts_sleep.tv_sec = 0;
        ts_sleep.tv_nsec = 100000000;

        // Connect to the server
        RCLCPP_INFO(this->get_logger(), "Connecting to Vicon Datastream server at %s", server_address.c_str());
        bool is_connected = false;
        for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) 
        {
            client->Connect(server_address);
            if(client->IsConnected().Connected) 
            {
                is_connected = true;
                break;
            }
            else
                nanosleep(&ts_sleep, NULL);
        }

        // Report if cannot connect
        if (!is_connected) 
        {
            RCLCPP_WARN(this->get_logger(),"Cannot Connect to Vicon server at %s", server_address.c_str());
            return false;
        }

        // Configure the connection
        RCLCPP_INFO(this->get_logger(),"Successfully Connect to Vicon server at %s", server_address.c_str());
        client->SetStreamMode(ViconSDK::StreamMode::ClientPull);
        client->SetAxisMapping(ViconSDK::Direction::Forward, ViconSDK::Direction::Left, ViconSDK::Direction::Up);
        client->EnableSegmentData();
        if(!client->IsSegmentDataEnabled().Enabled) 
        {
            RCLCPP_WARN(this->get_logger(),"Segment data cannot be enabled.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(),"Successfully configure Vicon server at %s", server_address.c_str());

        // Need to wait for some time after enabling data else you get junk frames
        //struct timespec ts_sleep;
        ts_sleep.tv_sec = 0;
        ts_sleep.tv_nsec = 100000000;
        nanosleep(&ts_sleep, NULL);

        // initialize the last_time for evaluate frame frequency
        rclcpp::Time t = now();
        last_time = t.seconds();
        
        return true;
    }


    void ViconDriver::run() 
    {
        ViconSDK::Result::Enum result = client->GetFrame().Result;
        if (result != ViconSDK::Result::Success)
            return;
        handleFrame();
        return;  
    }


    void ViconDriver::disconnect() 
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"Disconnected with the server at "<< server_address);
        client->Disconnect();
        return;  
    }


    void ViconDriver::handleFrame() 
    {
        
        int body_count = client->GetSubjectCount().SubjectCount;
        // Assign each subject with a thread
        vector<boost::thread> subject_threads;
        subject_threads.reserve(body_count);

        for (int i = 0; i< body_count; ++i) 
        {
            string subject_name = client->GetSubjectName(i).SubjectName;

            // Process the subject if required
            if (model_set.empty() || model_set.count(subject_name)) {
            // Create a new subject if it does not exist
            if (subjects.find(subject_name) == subjects.end()) {
                subjects[subject_name] = Subject::SubjectPtr(
                    new Subject(subject_name, fixed_frame_id));
                subjects[subject_name]->setParameters(
                    process_noise, measurement_noise, frame_rate);
            }
            // Handle the subject in a different thread
            subject_threads.emplace_back(&ViconDriver::handleSubject, this, i);
            //handleSubject(i);
            }
        }

        // Wait for all the threads to stop
        for(auto &thread : subject_threads) 
        {
            thread.join();
        }

        // Send out warnings
        for (auto it = subjects.begin(); it != subjects.end(); ++it) 
        {
            Subject::Status status = it->second->getStatus();
            if (status == Subject::LOST)
                RCLCPP_WARN(this->get_logger(), "Lose track of subject %s", (it->first).c_str());
            else if (status == Subject::INITIALIZING)
                RCLCPP_WARN(this->get_logger(),"Initialize subject %s", (it->first).c_str());
        }

        return;
        
    }


    void ViconDriver::handleSubject(const int& sub_idx) 
    {
        
        boost::unique_lock<boost::shared_mutex> write_lock(mtx);
        
        // We assume each subject has only one segment
        string subject_name = client->GetSubjectName(sub_idx).SubjectName;
        string segment_name = client->GetSegmentName(subject_name, 0).SegmentName;
        
        // Get the pose for the subject
        ViconSDK::Output_GetSegmentGlobalTranslation trans =
                    client->GetSegmentGlobalTranslation(subject_name, segment_name);
        ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat =
                    client->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
        write_lock.unlock();

        //boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        if(trans.Result != ViconSDK::Result::Success ||
            quat.Result != ViconSDK::Result::Success ||
            trans.Occluded || quat.Occluded) 
        {
            subjects[subject_name]->disable();
            return;
        }

        // Convert the msgs to Eigen type
        Eigen::Quaterniond m_att(quat.Rotation[3],
                    quat.Rotation[0], quat.Rotation[1], quat.Rotation[2]);
        Eigen::Vector3d m_pos(trans.Translation[0]/1000,
                    trans.Translation[1]/1000, trans.Translation[2]/1000);

        
        // Re-enable the object if it is lost previously
        if (subjects[subject_name]->getStatus() == Subject::LOST) 
        {
            subjects[subject_name]->enable();
        }

        // Feed the new measurement to the subject
        rclcpp::Time t2 = this->get_clock()->now();
        double time = t2.seconds();
        subjects[subject_name]->processNewMeasurement(time, m_att, m_pos);
        //read_lock.unlock();

        
        // Check the frequency
        counter++;
        if (counter == window_size)
        {
            double dt = (time-last_time)/window_size;
            if (dt > 1.2*frame_interval)
                RCLCPP_WARN(this->get_logger(),"The current odom frequency is %4.5f, less than %s of preset one, CAUTION", 1/dt, "80%");
            counter = 0;
            last_time = time;
        }

        
        // Publish tf if requred
        if (publish_tf && subjects[subject_name]->getStatus() == Subject::TRACKED) 
        {
            Quaterniond att = subjects[subject_name]->getAttitude();
            Vector3d pos = subjects[subject_name]->getPosition();

            rclcpp::Time t3 = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped stamped_transform;

            stamped_transform.header.stamp = t3;
            stamped_transform.header.frame_id = subject_name + "/" + fixed_frame_id;
            stamped_transform.child_frame_id = subject_name + "/base_link";

            stamped_transform.transform.translation.x = pos(0);
            stamped_transform.transform.translation.y = pos(1);
            stamped_transform.transform.translation.z = pos(2);

            stamped_transform.transform.rotation.w = att.w();
            stamped_transform.transform.rotation.x = att.x();
            stamped_transform.transform.rotation.y = att.y();
            stamped_transform.transform.rotation.z = att.z();

            write_lock.lock();                       
            tf_publisher->sendTransform(stamped_transform);
            write_lock.unlock();
        }
        
        return;
        
    }
    
}
