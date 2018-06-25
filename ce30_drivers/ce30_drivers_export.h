#ifndef CE30_VIZ_CE30_DRIVERS_EXPORT_H_
#define CE30_VIZ_CE30_DRIVERS_EXPORT_H_

#ifdef _WIN32
#	ifdef ce30_drivers_EXPORTS
#		define CE30_DRIVERS_API __declspec(dllexport)
#	else
#		define CE30_DRIVERS_API __declspec(dllimport)
#	endif
#else
#	define CE30_DRIVERS_API
#endif // WIN32

#endif // CE30_VIZ_CE30_DRIVERS_EXPORT_H_
