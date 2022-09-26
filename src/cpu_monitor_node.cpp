
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace ros2_computer_monitor
{
    class CPUMonitorNode : public rclcpp::Node
    {
        public :
            explicit CPUMonitorNode(const rclcpp::NodeOptions &);
            ~CPUMonitorNode();

        private :
            using CPUStatus = std::vector<std::vector<int>>;
            
            float m_diagnostic_period;

            float m_cpu_usage_warn_threshold,
                  m_cpu_usage_error_threshold;

            float m_cpu_speed_warn_threshold,
                  m_cpu_speed_error_threshold;

            double m_old_time_nanoseconds;

            std::vector<std::vector<int>> m_pre_cpu_core_times;

            std::vector<std::string> m_stat_column_names;

            std::unique_ptr<diagnostic_updater::Updater> m_diagnostic_updater;

            void cpuUsageDiagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper &);
            void cpuClockDiagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper &);

            void addCPUCoreTimeStatus
            (
                diagnostic_updater::DiagnosticStatusWrapper &,
                const double now_time_nanoseconds
            );
            void addCPUCoreClockSpeed(diagnostic_updater::DiagnosticStatusWrapper &);

            std::vector<float> getCPUUsage(
                const std::vector<std::vector<int>> &cpu_core_times,
                const double duration_time_milliseconds
            );

            void updatePreCPUCoreTimes(const std::vector<std::vector<int>> &cpu_core_times);
            void updateOldTimeNanoseconds(const double now_time_nanoseconds);
    };

    CPUMonitorNode::CPUMonitorNode(const rclcpp::NodeOptions &node_options)
        : rclcpp::Node("cpu_monitor_node", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "Start cpu_monitor_node");

        m_stat_column_names.push_back("user");
        m_stat_column_names.push_back("nice");
        m_stat_column_names.push_back("system");
        m_stat_column_names.push_back("idle");
        m_stat_column_names.push_back("iowait");
        m_stat_column_names.push_back("irq");
        m_stat_column_names.push_back("softirq");

        // TODO Remove hard coded parameter
        m_diagnostic_period = 1;
        m_cpu_usage_warn_threshold = 60;
        m_cpu_usage_error_threshold = 95;
        m_cpu_speed_warn_threshold =  3;
        m_cpu_speed_error_threshold =  4.5;

        m_diagnostic_updater = std::make_unique<diagnostic_updater::Updater>
        (
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_timers_interface(),
            this->get_node_topics_interface(),
            m_diagnostic_period
        );

        // TODO Add unique hardware id
        m_diagnostic_updater->setHardwareID("none");

        m_diagnostic_updater->add
        (
            "CPU Usage Status",
            std::bind
            (
                &CPUMonitorNode::cpuUsageDiagnosticsCallback,
                this,
                std::placeholders::_1
            )
        );
        m_diagnostic_updater->add
        (
            "CPU Clock Speed Status",
            std::bind
            (
                &CPUMonitorNode::cpuClockDiagnosticsCallback,
                this,
                std::placeholders::_1
            )
        );
    }

    CPUMonitorNode::~CPUMonitorNode()
    {
        RCLCPP_INFO(this->get_logger(), "Finish cpu_monitor_node");
    }

    void CPUMonitorNode::cpuUsageDiagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status)
    {
        const double now_time_nanoseconds = this->now().nanoseconds();

        addCPUCoreTimeStatus(diagnostic_status, now_time_nanoseconds);
    }

    void CPUMonitorNode::cpuClockDiagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status)
    {
        addCPUCoreClockSpeed(diagnostic_status);
    }

    void CPUMonitorNode::addCPUCoreTimeStatus
    (
        diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status,
        const double now_time_nanoseconds
    )
    {
        constexpr auto processor_status_filename = "/proc/stat";

        std::ifstream processor_status_file;

        try
        {
            processor_status_file.open
            (
                processor_status_filename,
                std::ios::in
            );
            if(!processor_status_file.is_open())
            {
                throw std::runtime_error("Failed open " + std::string(processor_status_filename));
            }
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
            return;
        }

        constexpr char processor_status_delimiter = ' ';
        constexpr int max_data_size = 4;

        const std::string cpu_time_tag = "cpu";

        std::string line_buffer,
                    element_buffer;

        std::vector<std::vector<int>> cpu_core_times;

        while(std::getline(processor_status_file, line_buffer))
        {
            bool first_element = true;
            int count_of_cpu_time = 0;

            std::istringstream iss(line_buffer);
            
            while(std::getline(iss, element_buffer, processor_status_delimiter))
            {
                // Empty element guard
                if(2 > element_buffer.size())
                {
                    continue;
                }
                // CPU name column process
                else if(std::exchange(first_element, false))
                {
                    if(std::string::npos == element_buffer.find(cpu_time_tag))
                    {
                        break;
                    }
                    cpu_core_times.push_back(std::vector<int>());
                    continue;
                }
                // Limit column data size
                else if(max_data_size <= count_of_cpu_time)
                {
                    break;
                }
                cpu_core_times.back().push_back
                (
                    std::atoi(element_buffer.c_str())
                );
                count_of_cpu_time ++;
            }
        }
        processor_status_file.close();

        if(m_pre_cpu_core_times.size() < 1)
        {
            for(const auto &core_times : cpu_core_times)
            {
                m_pre_cpu_core_times.push_back(std::vector<int>());
                m_pre_cpu_core_times.back().resize(core_times.size());
                std::copy
                (
                    core_times.cbegin(),
                    core_times.cend(),
                    m_pre_cpu_core_times.back().begin()
                );
            }
        }

        const double duration_time_milliseconds
            = static_cast<double>(now_time_nanoseconds - m_old_time_nanoseconds) * 1e-6;

        {
            int j = 0;
            bool first_call = true;
            float count_of_thread;

            for(const auto &core_times : cpu_core_times)
            {
                if(std::exchange(first_call, false))
                {
                    count_of_thread = cpu_core_times.size();
                }
                else
                {
                    count_of_thread = 1;
                }
                int i = 0;

                for(const auto &core_time : core_times)
                {
                    const std::string status_key = "core_" + std::to_string(j) + "_" + m_stat_column_names[i];
                    const float cpu_usage_percentage =
                        1e3 * (core_time - m_pre_cpu_core_times[j][i])
                        / (duration_time_milliseconds * count_of_thread);

                    diagnostic_status.add
                    (
                        status_key, std::to_string(cpu_usage_percentage) + " %"
                    );
                    i ++;
                }
                j ++;
            }
        }

        auto cpu_usage = getCPUUsage
        (
            cpu_core_times,
            duration_time_milliseconds
        );

        for(unsigned int i = 0; i < cpu_usage.size(); i ++)
        {
            diagnostic_status.add
            (
                "cpu_" + std::to_string(i) + "_usage",
                std::to_string(cpu_usage[i]) + " %"
            );
        }

        unsigned char status_msg;
        std::string usage_message;

        if(cpu_usage[0] > m_cpu_usage_error_threshold)
        {
            status_msg = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            usage_message = "Error";
        }
        else if(cpu_usage[0] > m_cpu_usage_warn_threshold)
        {
            status_msg = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            usage_message = "Warning";
        }
        else
        {
            status_msg = diagnostic_msgs::msg::DiagnosticStatus::OK;
            usage_message = "OK";
        }

        diagnostic_status.summary(status_msg, usage_message);

        updatePreCPUCoreTimes(cpu_core_times);
        updateOldTimeNanoseconds(now_time_nanoseconds);
    }

    void CPUMonitorNode::addCPUCoreClockSpeed(diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status)
    {
        constexpr auto cpu_info_filename = "/proc/cpuinfo";

        std::ifstream cpu_info_file;

        try
        {
            cpu_info_file.open
            (
                cpu_info_filename,
                std::ios::in
            );
            if(!cpu_info_file.is_open())
            {
                throw std::runtime_error("Failed open " + std::string(cpu_info_filename));
            }
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
        }

        constexpr char cpu_info_delimiter = ':';

        const std::string cpu_clock_speed_tag = "cpu MHz";

        std::string line_buffer,
                    element_buffer;

        std::vector<float> cpu_clock_speed;

        while(std::getline(cpu_info_file, line_buffer))
        {
            if(std::string::npos == line_buffer.find(cpu_clock_speed_tag))
            {
                continue;
            }

            bool first_element = true;
            std::istringstream iss(line_buffer);

            while(std::getline(iss, element_buffer, cpu_info_delimiter))
            {
                if(std::exchange(first_element, false))
                {
                    continue;
                }
                cpu_clock_speed.push_back
                (
                    std::atof(element_buffer.c_str()) * 1e-3
                );
            }
        }
        cpu_info_file.close();

        {
            int i = 0;

            for(const auto &clock_speed : cpu_clock_speed)
            {
                const std::string status_key = "core_" + std::to_string(i) + "_speed";

                diagnostic_status.add
                (
                    status_key,
                    std::to_string(clock_speed) + " GHz"
                );

                i ++;
            }
        }

        float sum_clock_speed = 0;

        for(const auto &clock_speed : cpu_clock_speed)
        {
            sum_clock_speed += clock_speed;
        }

        const float average_clock_speed = sum_clock_speed / cpu_clock_speed.size();

        unsigned char status_msg;
        std::string clock_status_message;

        if(average_clock_speed >= m_cpu_speed_error_threshold)
        {
            status_msg = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            clock_status_message = "Error";
        }
        else if(average_clock_speed >= m_cpu_speed_warn_threshold)
        {
            status_msg = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            clock_status_message = "Warning";
        }
        else
        {
            status_msg = diagnostic_msgs::msg::DiagnosticStatus::OK;
            clock_status_message = "OK";
        }

        diagnostic_status.add
        (
            "cpu_average_speed",
            std::to_string(average_clock_speed) + " GHz"
        );
        diagnostic_status.summary(status_msg, clock_status_message);
    }

    std::vector<float> CPUMonitorNode::getCPUUsage
    (
        const std::vector<std::vector<int>> &cpu_core_times,
        const double duration_time_milliseconds
    )
    {
        constexpr int usage_time_range = 3;

        std::vector<float> cpu_usage;

        for(unsigned int j = 0; j < cpu_core_times.size(); j ++)
        {
            float sum_cpu_time = 0;

            for(auto i = 0; i < usage_time_range; i ++)
            {
                sum_cpu_time += cpu_core_times[j][i] - m_pre_cpu_core_times[j][i];
            }

            float usage_percentage;

            if(0 == j)
            {
                usage_percentage = sum_cpu_time / (duration_time_milliseconds * cpu_core_times.size());
            }
            else
            {
                usage_percentage = sum_cpu_time / (duration_time_milliseconds);
            }

            cpu_usage.push_back(usage_percentage * 1e3);
        }
        return cpu_usage;
    }

    void CPUMonitorNode::updatePreCPUCoreTimes
    (
        const std::vector<std::vector<int>> &cpu_core_times
    )
    {
        int i = 0;
        for(const auto &core_times : cpu_core_times)
        {
            std::copy
            (
                core_times.cbegin(),
                core_times.cend(),
                m_pre_cpu_core_times[i].begin()
            );
            i ++;
        }
    }

    void CPUMonitorNode::updateOldTimeNanoseconds
    (
        const double now_time_nanoseconds
    )
    {
        m_old_time_nanoseconds = now_time_nanoseconds;
    }
} // namespace ros2_computer_monitor

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_computer_monitor::CPUMonitorNode)

