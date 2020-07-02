

namespace ROS2.Utils
{
    public static class QosProfile
    {
        public enum Profile 
        {
            SENSOR_DATA = 0,
            PROFILE_PARAMETERS = 1,
            DEFAULT = 2,
            SERVICES_DEFAULT = 3,
            PARAMETER_EVENTS = 4,
            SYSTEM_DEFAULTS = 5,
        }
    }
}
