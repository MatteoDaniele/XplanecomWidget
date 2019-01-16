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
float deg2rad = pi/180;
float rad2deg = 1/deg2rad;

// socket properties definition
const char* Host = "127.0.0.1";
int socketPort = 9016;

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
XPLMDataRef OverrideJoystick = NULL;
XPLMDataRef FlightModelReactivate = NULL;

XPLMDataRef xLocal = NULL;
XPLMDataRef yLocal = NULL;
XPLMDataRef zLocal = NULL;

XPLMDataRef phiDataRef = NULL;
XPLMDataRef thetaDataRef = NULL;
XPLMDataRef psiDataRef = NULL;

// dataref creation for main rotor angles
XPLMDataRef blade_pitch_main = NULL;
XPLMDataRef blade_flap = NULL;
XPLMDataRef blade_lag = NULL;
XPLMDataRef blade_pitch_tail = NULL;
XPLMDataRef rotors_shaft_angles = NULL;

float customDataRefValue[5];

float initialAnglesValue[5] = {10};
/*
int GetDesiredCustomDataRef(void* inRefcon,
                           	float *              outValues,    // Can be NULL
                           	int                  inOffset,
                           	int                  inMax);

void SetDesiredCustomDataRef(void *               inRefcon,
                             float *              inValues,
                             int                  inOffset,
                             int                  inCount);
*/
/*
float GetPitchAnglesMain(void* inRefcon);
float SetPitchAnglesMain(void* inRefcon, float pitchValuesMain);
float GetFlapAnglesMain(void* inRefcon);
float SetFlapAnglesMain(void* inRefcon, float FlapValues);
float GetLagAnglesMain(void* inRefcon);
float SetLagAnglesMain(void* inRefcon, float LagValues);
float GetPitchAnglesTail(void* inRefcon);
float SetPitchAnglesTail(void* inRefcon, float pitchValuesTail);
*/

// initialize socket and structure
int socketInfo;
struct sockaddr_in server;
int receivedArray=1;

// socket function declaration
static float ReceiveDataFromSocket(
                                   float                inElapsedSinceLastCall,
                                   float                inElapsedTimeSinceLastFlightLoop,
                                   int                  inCounter,
                                   void *               inRefcon);

/* socket message will be a total of 3 doubles and 3+16 float
representing respectively latitude, longitude, elevation, phi, theta, psi and blades angles*/
// structure definition
struct{
double latitudeReceived;
double longitudeReceived;
double elevationReceived;
float phiReceived;
float thetaReceived;
float psiReceived;
float BLADE_1_pitch;
float BLADE_1_flap;
float BLADE_1_lag;
float BLADE_2_pitch;
float BLADE_2_flap;
float BLADE_2_lag;
float BLADE_3_pitch;
float BLADE_3_flap;
float BLADE_3_lag;
float BLADE_4_pitch;
float BLADE_4_flap;
float BLADE_4_lag;
float BLADE_5_pitch;
float BLADE_5_flap;
float BLADE_5_lag;
float TAIL_PITCH;
float MR_SHAFT_ANGLE;
float TR_SHAFT_ANGLE;
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
float thetaMain[5];
float betaMain[5];
float epsMain[5];
float thetaTail[5];
float rotorsShaftAngles[2];

double localX;
double localY;
double localZ;

// array to override internal fdm (20 values can be overridden, interested only in the first)
int deactivateFlag[1];
// array to reactivate internal fdm
int reactivateFlag[1];
// flag to check deactivation of fdm;
//int isFDMActiveFlag[1];
//int isFDMActive;
// button state for the widget
static XPWidgetID FDMbuttonState[1] = {NULL};

/*
// functions for custom datarefs
int GetDesiredCustomDataRef(void *               inRefcon,
                            float *              inValues,
                            int                  inOffset,
                            int                  inCount){

	return *inValues;
}

void SetDesiredCustomDataRef(void *               inRefcon,
                             float *              inValues,
                             int                  inOffset,
                             int                  inCount){

	for(int i=inOffset;i==inCount;i++ ){
	customDataRefValue[i] = inValues[i];
	}
}
*/

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
  std::cout << "Listening from IP address: "<< Host <<", port: "<< socketPort <<" ..." << '\n';
	return 0;
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

	thetaMain[0] = positionAndAttitude_.BLADE_1_pitch;
	betaMain[0] = positionAndAttitude_.BLADE_1_flap;
	epsMain[0] = positionAndAttitude_.BLADE_1_lag;
	thetaMain[1] = positionAndAttitude_.BLADE_2_pitch;
	betaMain[1] = positionAndAttitude_.BLADE_2_flap;
	epsMain[1] = positionAndAttitude_.BLADE_2_lag;
	thetaMain[2] = positionAndAttitude_.BLADE_3_pitch;
	betaMain[2] = positionAndAttitude_.BLADE_3_flap;
	epsMain[2] = positionAndAttitude_.BLADE_3_lag;
	thetaMain[3] = positionAndAttitude_.BLADE_4_pitch;
	betaMain[3] = positionAndAttitude_.BLADE_4_flap;
	epsMain[3] = positionAndAttitude_.BLADE_4_lag;
	thetaMain[4] = positionAndAttitude_.BLADE_5_pitch;
	betaMain[4] = positionAndAttitude_.BLADE_5_flap;
	epsMain[4] = positionAndAttitude_.BLADE_5_lag;
	// for now only the collective value of the tail is provided, unnecessary to be more specific
	thetaTail[0] = positionAndAttitude_.TAIL_PITCH;
	thetaTail[1] = positionAndAttitude_.TAIL_PITCH;
	thetaTail[2] = positionAndAttitude_.TAIL_PITCH;
	thetaTail[3] = positionAndAttitude_.TAIL_PITCH;
	// main and tail rotor RPM
	rotorsShaftAngles[0] = positionAndAttitude_.MR_SHAFT_ANGLE;
	rotorsShaftAngles[1] = positionAndAttitude_.TR_SHAFT_ANGLE;


	float elapsedTime = XPLMGetElapsedTime();
  // print what is received
  std::cout << "bytes received= "<< receivedArray << '\n';
	std::cout << "elapsed time [sec] =" << elapsedTime << '\n';
/*
	std::cout << "latitude [deg]= "<< latitude << '\n';
  std::cout << "longitude [deg]= "<< longitude << '\n';
  std::cout << "elevation [m]= "<< elevation << '\n';
  std::cout << "phi [deg]= "<< phi << '\n';
  std::cout << "theta [deg]= "<< theta << '\n';
  std::cout << "psi [deg]= "<< psi << '\n';
*/

/*
	std::cout << "theta[1] [deg]= "<< thetaMain[0] << '\n';
	std::cout << "theta[2] [deg]= "<< thetaMain[1] << '\n';
	std::cout << "theta[3] [deg]= "<< thetaMain[2] << '\n';
	std::cout << "theta[4] [deg]= "<< thetaMain[3] << '\n';
	std::cout << "theta[5] [deg]= "<< thetaMain[4] << '\n';

	std::cout << "beta[1] [deg]= "<< betaMain[0] << '\n';
	std::cout << "beta[2] [deg]= "<< betaMain[1] << '\n';
	std::cout << "beta[3] [deg]= "<< betaMain[2] << '\n';
	std::cout << "beta[4] [deg]= "<< betaMain[3] << '\n';
	std::cout << "beta[5] [deg]= "<< betaMain[4] << '\n';

	std::cout << "eps[1] [deg]= "<< epsMain[0] << '\n';
	std::cout << "eps[2] [deg]= "<< epsMain[1] << '\n';
	std::cout << "eps[3] [deg]= "<< epsMain[2] << '\n';
	std::cout << "eps[4] [deg]= "<< epsMain[3] << '\n';
	std::cout << "eps[5] [deg]= "<< epsMain[4] << '\n';

	std::cout << "thetaTail[1] [deg]= "<< thetaTail[0] << '\n';
	std::cout << "thetaTail[2] [deg]= "<< thetaTail[1] << '\n';
	std::cout << "thetaTail[3] [deg]= "<< thetaTail[2] << '\n';
	std::cout << "thetaTail[4] [deg]= "<< thetaTail[3] << '\n';
*/

	std::cout << "MR_SHAFT_ANGLE [deg]= "<< rotorsShaftAngles[0] << '\n';
	std::cout << "TR_SHAFT_ANGLE [deg]= "<< rotorsShaftAngles[1] << '\n';

	//transform in local coordinated that can be written as datarefs
	XPLMWorldToLocal(latitude,longitude,elevation,&localX,&localY,&localZ);


	// set the useful data
	XPLMSetDatad(xLocal,localX);
	XPLMSetDatad(yLocal,localY);
	XPLMSetDatad(zLocal,localZ);
	XPLMSetDataf(phiDataRef,phi);
	XPLMSetDataf(thetaDataRef,theta);
	XPLMSetDataf(psiDataRef,psi);

	XPLMSetDatavf(blade_pitch_main,thetaMain,0,5);
	XPLMSetDatavf(blade_flap,betaMain,0,5);
	XPLMSetDatavf(blade_lag,epsMain,0,5);
	XPLMSetDatavf(blade_pitch_tail,thetaTail,0,5);
	XPLMSetDatavf(rotors_shaft_angles,rotorsShaftAngles,0,2);


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
	strcpy(outDesc, "A plugin that disables Xplane's FDM in order to work as visualizer for AttilaFrameSim, by Matteo Daniele");

	// create the menu
	// sub menu element (item of MenuXPlaneCom)
	MenuItemXPlaneCom = XPLMAppendMenuItem(XPLMFindPluginsMenu(),"Internal FDM deactivator",NULL,1);
	// root menu element
	MenuXPlaneCom = XPLMCreateMenu("Internal FDM deactivator", XPLMFindPluginsMenu(),MenuItemXPlaneCom,InstructionsMenuHandler,NULL);
	XPLMAppendMenuItem(MenuXPlaneCom,"activation & network settings",(void *) +1, 1);


	/* NOTE: THE FOLLOWING LINES ARE RESPONSIBLE FOR THE DEACTIVATION
		 OF THE INTERNAL FDM */
	//isFDMActiveFlag[0]=0;
	//isFDMActive = XPLMGetDatavi(OverrideFDMDataRef,isFDMActiveFlag,0,1);
	// get dataref handles when plugin starts
	deactivateFlag[0] = 1;
	OverrideFDMDataRef = XPLMFindDataRef("sim/operation/override/override_planepath");
	OverrideJoystick = XPLMFindDataRef("sim/operation/override/override_joystick");
	// find the 6 useful data
	xLocal 	= XPLMFindDataRef("sim/flightmodel/position/local_x");
	yLocal  = XPLMFindDataRef("sim/flightmodel/position/local_y");
	zLocal  = XPLMFindDataRef("sim/flightmodel/position/local_z");

	// euler angles
	phiDataRef 	= XPLMFindDataRef("sim/flightmodel/position/phi");
	thetaDataRef  = XPLMFindDataRef("sim/flightmodel/position/theta");
	psiDataRef  = XPLMFindDataRef("sim/flightmodel/position/psi");

/*
	// create custom dataref to set flap, lag, pitch, angles of the blades
	blade_pitch_main = XPLMRegisterDataAccessor("rotors/main/pitch-deg",
																					xplmType_FloatArray,                                // The types we support
																					1,                                             // Writable
																					NULL, NULL,															       // Integer accessors
																					NULL, NULL,                                    // Float accessors
																					NULL, NULL,                                    // Doubles accessors
																					NULL, NULL,                                    // Int array accessors
																					GetDesiredCustomDataRef, SetDesiredCustomDataRef,	// Float array accessors
																					NULL, NULL,
																					NULL, NULL);                                   // Refcons not used);

	blade_flap = XPLMRegisterDataAccessor("rotors/main/flap-deg",
																					xplmType_FloatArray,                                // The types we support
																					1,                                             // Writable
																					NULL, NULL,															       // Integer accessors
																					NULL, NULL,                                    // Float accessors
																					NULL, NULL,                                    // Doubles accessors
																					NULL, NULL,                                    // Int array accessors
																					GetDesiredCustomDataRef, SetDesiredCustomDataRef,	// Float array accessors
																					NULL, NULL,
																					NULL, NULL);
	blade_lag = XPLMRegisterDataAccessor("rotors/main/lag-deg",
																					xplmType_FloatArray,                                // The types we support
																					1,                                             // Writable
																					NULL, NULL,															       // Integer accessors
																					NULL, NULL,                                    // Float accessors
																					NULL, NULL,                                    // Doubles accessors
																					NULL, NULL,                                    // Int array accessors
																					GetDesiredCustomDataRef, SetDesiredCustomDataRef,	// Float array accessors
																					NULL, NULL,
																					NULL, NULL);
	blade_pitch_tail = XPLMRegisterDataAccessor("rotors/tail/pitch-deg",
																					xplmType_FloatArray,                                // The types we support
																					1,                                             // Writable
																					NULL, NULL,															       // Integer accessors
																					NULL, NULL,                                    // Float accessors
																					NULL, NULL,                                    // Doubles accessors
																					NULL, NULL,                                    // Int array accessors
																					GetDesiredCustomDataRef, SetDesiredCustomDataRef,	// Float array accessors
																					NULL, NULL,
																					NULL, NULL);
*/

  // find and initialize the newly created customDataRefs
/*
	blade_pitch_main = XPLMFindDataRef("rotors/main/pitch-deg");
	blade_flap = XPLMFindDataRef("rotors/main/flap-deg");
	blade_lag = XPLMFindDataRef("rotors/main/lag-deg");
	blade_pitch_tail = XPLMFindDataRef("rotors/tail/pitch-deg");
*/

	// array datarefs available and not used in our case
	blade_pitch_main = XPLMFindDataRef("sim/flightmodel2/wing/elevator1_deg");
	blade_flap = XPLMFindDataRef("sim/flightmodel2/wing/flap1_deg");
	blade_lag = XPLMFindDataRef("sim/flightmodel2/wing/rudder1_deg");
	blade_pitch_tail = XPLMFindDataRef("sim/flightmodel2/wing/elevator2_deg");
	rotors_shaft_angles = XPLMFindDataRef("sim/flightmodel2/engines/prop_rotation_angle_deg");

	XPLMSetDatavf(blade_pitch_main,initialAnglesValue,0,5);
	XPLMSetDatavf(blade_flap,initialAnglesValue,0,5);
	XPLMSetDatavf(blade_lag,initialAnglesValue,0,5);
	XPLMSetDatavf(blade_pitch_tail,initialAnglesValue,0,4);
	XPLMSetDatavf(rotors_shaft_angles,initialAnglesValue,0,2);


	// register flightloop callback
	XPLMRegisterFlightLoopCallback(ReceiveDataFromSocket,dt,NULL);

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

	/* Unregister data accessors*/
	/*
	XPLMUnregisterDataAccessor(blade_pitch_main);
	XPLMUnregisterDataAccessor(blade_flap);
	XPLMUnregisterDataAccessor(blade_lag);
	XPLMUnregisterDataAccessor(blade_pitch_tail);*/

	/* Unregister the callback */
	XPLMUnregisterFlightLoopCallback(ReceiveDataFromSocket, NULL);

}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }
/*
float OverrideLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	return 1;
}
*/
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

		if(XPGetWidgetProperty(FDMbuttonState[0],xpProperty_ButtonState,NULL)==1){

		CreateSocket(Host, socketPort);
		// start receiving data from socket
		// here comes the override of the internal fdm
		XPLMSetDatavi(OverrideFDMDataRef,deactivateFlag,0,1);
		// here comes joystick override
		XPLMSetDatai(OverrideJoystick,1);


  	return 0;
	}/*
		// reactivate the internal fdm and the close the socket when the button is pushed again
		else{
			//TODO: plugin stop OR plugin disable to reactivate the internal FDM?
			reactivateFlag[0]=0;
			XPLMSetDatavi(OverrideFDMDataRef,reactivateFlag,0,1);
			close(socketInfo);
			return 0;
		}*/
	}

	return 0;

}
