#ifndef CE30_PCVIZ_EXPORT_H
#define CE30_PCVIZ_EXPORT_H

#ifdef _WIN32
#	ifdef ce30_pcviz_EXPORTS
#		define API __declspec(dllexport)
#	else
#		define API __declspec(dllimport)
#	endif
#else
#	define API
#endif // WIN32

#endif // CE30_PCVIZ_EXPORT_H
