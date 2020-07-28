
using System;
using System.Runtime.InteropServices;
using ROS2.Utils;

namespace ROS2 {
    public class TransformListener {
        private static readonly DllLoadUtils dllLoadUtils;

        static TransformListener()
        {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLib = dllLoadUtils.LoadLibrary("rclcppdotnet");

            IntPtr native_rclcpp_init_ptr = dllLoadUtils.GetProcAddress(nativeLib, "native_rclcpp_init");
            TransformListener.native_rclcpp_init = (NativeRclcppInitType)Marshal.GetDelegateForFunctionPointer(
                native_rclcpp_init_ptr, typeof(NativeRclcppInitType));

            IntPtr nativeLibTFL = dllLoadUtils.LoadLibrary("transform_listener");

            IntPtr native_construct_buffer_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_construct_buffer");
            TransformListener.native_construct_buffer = (NativeConstructBufferType)Marshal.GetDelegateForFunctionPointer(
                native_construct_buffer_ptr, typeof(NativeConstructBufferType));

            IntPtr native_construct_listener_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_construct_listener");
            TransformListener.native_construct_listener = (NativeConstructListenerType)Marshal.GetDelegateForFunctionPointer(
                native_construct_listener_ptr, typeof(NativeConstructListenerType));

            IntPtr native_construct_time_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_construct_time");
            TransformListener.native_construct_time = (NativeConstructTimeType)Marshal.GetDelegateForFunctionPointer(
                native_construct_time_ptr, typeof(NativeConstructTimeType));

            IntPtr native_lookup_transform_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_lookup_transform");
            TransformListener.native_lookup_transform = (NativeLookupTransformType)Marshal.GetDelegateForFunctionPointer(
                native_lookup_transform_ptr, typeof(NativeLookupTransformType));

            IntPtr native_retrieve_translation_z_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_retrieve_translation_z");
            TransformListener.native_retrieve_translation_z = (NativeRetrieveTranslationZType)Marshal.GetDelegateForFunctionPointer(
                native_retrieve_translation_z_ptr, typeof(NativeRetrieveTranslationZType));

        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void NativeRclcppInitType();
        private static NativeRclcppInitType native_rclcpp_init = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate IntPtr NativeConstructBufferType();
        private static NativeConstructBufferType native_construct_buffer = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate IntPtr NativeConstructListenerType(IntPtr buf);
        private static NativeConstructListenerType native_construct_listener = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate IntPtr NativeConstructTimeType(int sec, int nano);
        private static NativeConstructTimeType native_construct_time = null;

        /*
        [DllImport("transform_listener.dll", EntryPoint = "native_lookup_transform")]
        private static extern IntPtr native_lookup_transform(IntPtr buf,
            [MarshalAs(UnmanagedType.LPStr)] string from,
            [MarshalAs(UnmanagedType.LPStr)] string to,
            IntPtr t); */

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate IntPtr NativeLookupTransformType(IntPtr buf, string from, string to, IntPtr t);
        private static NativeLookupTransformType native_lookup_transform = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate double NativeRetrieveTranslationZType(IntPtr f);
        private static NativeRetrieveTranslationZType native_retrieve_translation_z = null;

        public TransformListener()
        {
            native_rclcpp_init();
            IntPtr buf = native_construct_buffer();
            IntPtr listener = native_construct_listener(buf);
            IntPtr transformHandle = native_lookup_transform(buf, "foo", "bar",
                native_construct_time(0, 0));
            native_retrieve_translation_z(transformHandle);
        }
    }
}