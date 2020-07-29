
using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
    internal class RclCppDotnetDelegates {
        internal static readonly DllLoadUtils dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void NativeRclcppInitType();
        internal static NativeRclcppInitType native_rclcpp_init = null;

        static RclCppDotnetDelegates() {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr pDll = dllLoadUtils.LoadLibrary("rclcppdotnet");

            IntPtr native_rclcpp_init_ptr = dllLoadUtils.GetProcAddress(nativeLib, "native_rclcpp_init");
            RclCppDotnetDelegates.native_rclcpp_init = (NativeRclcppInitType)Marshal.GetDelegateForFunctionPointer(
                native_rclcpp_init_ptr, typeof(NativeRclcppInitType));
        }
    }

    public class RclCppDotnet {
        private static bool initialized = false;
        private static readonly object syncLock = new object();

        public static void Init()
        {
            lock (syncLock)
            {
                if (!initialized)
                {
                    RclCppDotnetDelegates.native_rclcpp_init();
                    initialized = true;
                }
            }
        }
    }
}