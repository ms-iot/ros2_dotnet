#ifndef RCL_DOTNET_TFL
#define RCL_DOTNET_TFL

#ifdef __cplusplus
extern "C" {
#endif

	__declspec(dllexport)
	void* __cdecl native_construct_buffer();

	__declspec(dllexport)
	void* __cdecl native_construct_listener(void* buf);

	__declspec(dllexport)
	void* __cdecl native_construct_time(int sec, int nano);

	__declspec(dllexport)
	void* __cdecl native_lookup_transform(void* buf,
		char* from, char* to, void* t);

	__declspec(dllexport)
	double __cdecl native_retrieve_translation_x(void* tf);

	__declspec(dllexport)
	double __cdecl native_retrieve_translation_y(void* tf);

	__declspec(dllexport)
	double __cdecl native_retrieve_translation_z(void* tf);

#ifdef __cplusplus
}
#endif


#endif // RCL_DOTNET_TFL