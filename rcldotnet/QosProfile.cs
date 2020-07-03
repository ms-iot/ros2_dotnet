

namespace ROS2.Utils
{
    public static class QosProfile
    {
        /// <summary> QoS presets that encompass the majority of use-cases for ROS2 messages
        /// for more details on the different policies and profile use cases, see:
        /// https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/ </summary>
        public enum Profile 
        {
            /// <summary> Prioritizes timely readings, rather than 
            /// ensuring they all arrive. Best effort reliability and smaller queue depth. </summary>
            SENSOR_DATA = 0,
            /// <summary> For use in services. Has large queue depth so that requests are not lost
            /// when waiting for client/server to respond. </summary>
            PROFILE_PARAMETERS = 1,
            /// <summary> The most similar profile to ROS1 behavior. Reliable, volatile durability, "keep last" history. </summary>
            DEFAULT = 2,
            /// <summary> For use in services. Reliable, with volatile durability to
            /// ensure that service servers who restart don't recieve outdated requests </summary>
            SERVICES_DEFAULT = 3,
            /// <summary> Has a large queue depth to ensure requests are not lost. </summary>
            PARAMETER_EVENTS = 4,
            /// <summary> Uses the RMW implementation's default values. </summary>
            SYSTEM_DEFAULTS = 5,
        }
    }
}
