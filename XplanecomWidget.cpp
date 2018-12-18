#include <XPLMGraphics.h>
#include <XPLMDisplay.h>

#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include <XPLMDataAccess.h>
#include <iostream>
#include <cstdio>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>


#if IBM
	#include <windows.h>
#endif
#if LIN
	#include <GL/gl.h>
#elif __GNUC__
	#include <OpenGL/gl.h>
#else
	#include <GL/gl.h>
#endif

#ifndef XPLM300
	#error This is made to be compiled against the XPLM300 SDK
#endif

// variables needed for the simulator
float dt = -1;//0.006;
float pi = 3.14159265359;


char InstructionsText[50][200] = {
"  Network options",
"  Host address",
"  Port",
"  FDM internal/external",
""
};

// initialize widget
XPWidgetID	ExternalFDMWidget = NULL;
XPWidgetID	InstructionsWindow = NULL;
XPWidgetID	InstructionsTextWidget[50] = {NULL};

void InstructionsMenuHandler(void *, void *);
void CreateWidget(int x1, int y1, int w, int h);
int InstructionsHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, long  inParam1, long  inParam2);
int gMenuItem;

// initialize the datarefs
XPLMDataRef OverrideFDMDataRef = NULL;
XPLMDataRef FlightModelReactivate = NULL;

XPLMDataRef xLocal = NULL;
XPLMDataRef yLocal = NULL;
XPLMDataRef zLocal = NULL;
/*
XPLMDataRef phiFromDataRef = NULL;
XPLMDataRef thetaFromDataRef = NULL;
XPLMDataRef psiFromDataRef = NULL;
*/
// orientation given in quaternions
XPLMDataRef quaternionVector = {NULL};

// initialize socket and structure
int socketInfo;
struct sockaddr_in server;
int receivedArray;
// socket properties definition
const char* Host;
int socketPort;
// socket function declaration
static float ReceiveDataFromSocket(
                                   float                inElapsedSinceLastCall,
                                   float                inElapsedTimeSinceLastFlightLoop,
                                   int                  inCounter,
                                   void *               inRefcon);

/* socket message will be a total of 3 doubles and 3 float
representing respectively latitude, longitude, elevation, phi, theta, psi */
// structure definition
struct{
double latitudeReceived;
double longitudeReceived;
double elevationReceived;
float phiReceived;
float thetaReceived;
float psiReceived;
}
// positionAndAttitude_ is a structure
positionAndAttitude_;

// values initialization
double latitude;
double longitude;
double elevation;
float phi;
float theta;
float psi;

double localX;
double localY;
double localZ;

// for quaternion calculation
float qVec[4];
float phi1;
float theta1;
float psi1;

// array to override internal fdm (20 values can be overridden, interested only in the first)
int deactivateFlag[1];
// array to reactivate internal fdm
int reactivateFlag[1];
// flag to check deactivation of fdm;
int isFDMActiveFlag[1];
int isFDMActive;
// button state for the widget
static XPWidgetID FDMbuttonState[1] = {NULL};


//socket functions
int CreateSocket(const char* host, int socketport){
	//create socket
  socketInfo = socket(AF_INET,SOCK_DGRAM,0);
  // assign values
  server.sin_addr.s_addr = inet_addr(Host);
  server.sin_family = AF_INET;
  server.sin_port = htons(socketPort);
  // bind to socket
  if(bind(socketInfo,(struct sockaddr *) &server,sizeof(server))<0){
    std::cerr<< "ERROR ON BINDING"<<'\n';
    return 1;
  }
  std::cout << "Listening from IP address: "<< Host <<", port: "<< socketPort << '\n';
}

float ReceiveDataFromSocket(       float                inElapsedSinceLastCall,
                                   float                inElapsedTimeSinceLastFlightLoop,
                                   int                  inCounter,
                                   void *               inRefcon){

  // receive incoming message
  receivedArray = recv(socketInfo,&positionAndAttitude_, sizeof(positionAndAttitude_), 0);
  if( receivedArray < 0) {
    std::cerr<< "ERROR ON RECEIVING"<<'\n';
		std::cout << "bytes received= "<< receivedArray << '\n';
    return 1;
    }

  // transform the message into something useful
  latitude = positionAndAttitude_.latitudeReceived;
  longitude = positionAndAttitude_.longitudeReceived;
  elevation = positionAndAttitude_.elevationReceived;
  phi = positionAndAttitude_.phiReceived;
  theta = positionAndAttitude_.thetaReceived;
  psi = positionAndAttitude_.psiReceived;


	float elapsedTime = XPLMGetElapsedTime();
  // print what is received
  std::cout << "bytes received= "<< receivedArray << '\n';
	std::cout << "elapsed time [sec] =" << elapsedTime << '\n';
  std::cout << "latitude [deg]= "<< latitude << '\n';
  std::cout << "longitude [deg]= "<< longitude << '\n';
  std::cout << "elevation [m]= "<< elevation << '\n';
  std::cout << "phi [deg]= "<< phi << '\n';
  std::cout << "theta [deg]= "<< theta << '\n';
  std::cout << "psi [deg]= "<< psi << '\n';

	//transform in local coordinated that can be written as datarefs
	XPLMWorldToLocal(latitude,longitude,elevation,&localX,&localY,&localZ);

	// transform the desired flight angles in quaternions
	phi1   = pi/360.0*phi;
	theta1 = pi/360.0*theta;
	psi1   = pi/360.0*psi;
	qVec[0] =  cos(psi1) * cos(theta1) * cos(phi1) + sin(psi1) * sin(theta1) * sin(phi1);
	qVec[1] =  cos(psi1) * cos(theta1) * sin(phi1) - sin(psi1) * sin(theta1) * cos(phi1);
	qVec[2] =  cos(psi1) * sin(theta1) * cos(phi1) + sin(psi1) * cos(theta1) * sin(phi1);
	qVec[3] = -cos(psi1) * sin(theta1) * sin(phi1) + sin(psi1) * cos(theta1) * cos(phi1);


	// set the useful data
	XPLMSetDatad(xLocal,localX);
	XPLMSetDatad(yLocal,localY);
	XPLMSetDatad(zLocal,localZ);
	XPLMSetDatavf(quaternionVector,qVec,0,4);
	return dt;

}


PLUGIN_API int XPluginStart(
							char *		outName,
							char *		outSig,
							char *		outDesc)
{

	XPLMMenuID  MenuXPlaneCom;
	int 				MenuItemXPlaneCom;
	strcpy(outName, "Xplanecom");
	strcpy(outSig, "AttilaFrameSim.UDPplugin.Xplanecom");
	strcpy(outDesc, "A plugin that disables Xplane's FDM, by Matteo Daniele");

	// register flightloop callback
	XPLMRegisterFlightLoopCallback(ReceiveDataFromSocket,dt,NULL);

	// create the menu
	// sub menu element (item of MenuXPlaneCom)
	MenuItemXPlaneCom = XPLMAppendMenuItem(XPLMFindPluginsMenu(),"Internal FDM deactivator",NULL,1);
	// root menu element
	MenuXPlaneCom = XPLMCreateMenu("Internal FDM deactivator", XPLMFindPluginsMenu(),MenuItemXPlaneCom,InstructionsMenuHandler,NULL);
	XPLMAppendMenuItem(MenuXPlaneCom,"activation & network settings",(void *) +1, 1);


	/* NOTE: THE FOLLOWING LINES ARE RESPONSIBLE FOR THE DEACTIVATION
		 OF THE INTERNAL FDM */
	isFDMActiveFlag[0]=0;
	isFDMActive = XPLMGetDatavi(OverrideFDMDataRef,isFDMActiveFlag,0,1);
	// get dataref handles when plugin starts
	deactivateFlag[0] = 1;
	OverrideFDMDataRef = XPLMFindDataRef("sim/operation/override/override_planepath");
	// find the 6 useful data
	xLocal 	= XPLMFindDataRef("sim/flightmodel/position/local_x");
	yLocal  = XPLMFindDataRef("sim/flightmodel/position/local_y");
	zLocal  = XPLMFindDataRef("sim/flightmodel/position/local_z");

	quaternionVector = XPLMFindDataRef("sim/flightmodel/position/q");
	//TODO: better work with quaternions since euler angles are not reported so precisely
	/*
	phiFromDataRef 			= XPLMFindDataRef("sim/flightmodel/position/phi");
	thetaFromDataRef 		= XPLMFindDataRef("sim/flightmodel/position/theta");
	psiFromDataRef 			= XPLMFindDataRef("sim/flightmodel/position/psi");
	*/
	// flag tells us if the widget is displayed
	gMenuItem = 0;
	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	//we'll be good citizens and clean it up
	if (gMenuItem == 1)
	{
		// destroy when closing
		XPDestroyWidget(ExternalFDMWidget,1);
	}

	//TODO: plugin stop OR plugin disable to reactivate the internal FDM?
	reactivateFlag[0]=0;
	XPLMSetDatavi(OverrideFDMDataRef,reactivateFlag,0,1);
	close(socketInfo);
	/* Unregister the callback */
	XPLMUnregisterFlightLoopCallback(ReceiveDataFromSocket, NULL);

}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }

void InstructionsMenuHandler(void * inMenuRef, void * inItemRef)
{
	switch ( (uintptr_t) inItemRef)
	{

		case 1:	if (gMenuItem == 0)
				{
					CreateWidget(50, 256, 256, 128);	//left, top, right, bottom.
					gMenuItem = 1;
				}
				else
				{
					if(!XPIsWidgetVisible(ExternalFDMWidget))
					XPShowWidget(ExternalFDMWidget);
				}
				break;
	}
}

// This will create our widget dialog.
void CreateWidget(int x, int y, int w, int h)
{
int Index;
int x2 = x + w;
int y2 = y - h;

// Create the Main Widget window.
	ExternalFDMWidget = XPCreateWidget(x, y, x2, y2,
					1,										// Visible
					"External FDM Widget for AttilaFrameSim",	// desc
					1,										// root
					NULL,									// no container
					xpWidgetClass_MainWindow);
					// Add Close Box to the Main Widget.  Other options are available.  See the SDK Documentation.
					XPSetWidgetProperty(ExternalFDMWidget, xpProperty_MainWindowHasCloseBoxes, 1);


					// Print each line of instructions.
						for (Index=0; Index < 50; Index++)
						{
						if(strcmp(InstructionsText[Index],"") == 0) {break;}

							// Create a text widget
							InstructionsTextWidget[Index] = XPCreateWidget(x+10, y-(30+(Index*20)) , x2-20, y-(42+(Index*20)),
							1,	// Visible
							InstructionsText[Index],// desc
							0,		// root
							ExternalFDMWidget,
							xpWidgetClass_Caption);

							if(strcmp(InstructionsText[Index],"  FDM internal/external")==0)
							{
								// create button for activation or deactivation of FDM
								FDMbuttonState[0] = XPCreateWidget(x+50, y-(30+(Index*20)) , x2-10, y-(42+(Index*20)),
								1,
								"",
								0,
								ExternalFDMWidget,
								xpWidgetClass_Button);
								// set it to be a checkbox
								XPSetWidgetProperty(FDMbuttonState[0], xpProperty_ButtonType, xpRadioButton);
								XPSetWidgetProperty(FDMbuttonState[0], xpProperty_ButtonBehavior, xpButtonBehaviorCheckBox);
								XPSetWidgetProperty(FDMbuttonState[0], xpProperty_ButtonState, 0);
							}

						}




					// Register our widget handler
					XPAddWidgetCallback(ExternalFDMWidget, InstructionsHandler);

}



// This is our widget handler.
int	InstructionsHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, long  inParam1, long  inParam2)
{




	if (inMessage == xpMessage_CloseButtonPushed)
	{
		if (gMenuItem == 1)
		{
			XPHideWidget(ExternalFDMWidget);
		}
		return 1;
	}

	// handle any checkbox selection
	if (inMessage == xpMsg_ButtonStateChanged)
	{

		receivedArray = 1;
		// here comes the override of the internal fdm
		XPLMSetDatavi(OverrideFDMDataRef,deactivateFlag,0,1);
		// socket properties definition
		Host = "127.0.0.1";
		socketPort = 8888;
		CreateSocket(Host, socketPort);
		// start receiving data from socket
  	// ReceiveDataFromSocket(Host,socketPort, receivedArray);
		return 0;
	}

	return 0;

}
