#include "kernel.h"
#include <circle/startup.h>

int main (void)
{
	// cannot return here because some destructors used in CMultiSynthLoader are not implemented

	CKernel kernel;
	if (!kernel.Initialize ())
	{
		halt ();
		return EXIT_HALT;
	}
	
	CStdlibApp::TShutdownMode ShutdownMode = kernel.Run ();

	kernel.Cleanup ();

	switch (ShutdownMode)
	{
	case CStdlibApp::ShutdownReboot:
		reboot ();
		return EXIT_REBOOT;

	case CStdlibApp::ShutdownHalt:
	default:
		halt ();
		return EXIT_HALT;
	}
}