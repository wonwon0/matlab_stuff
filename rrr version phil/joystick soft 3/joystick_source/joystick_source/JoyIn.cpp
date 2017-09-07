// Joyin.cpp : Implementation of CJoyin class
// Copyright 1998-2000 by The MathWorks, Inc.
// $Revision: 1 $  $Date: 1/23/01 3:22p $


#include "stdafx.h"
#include "Joy.h"
#include "Joyin.h"
#include "Joyadapt.h"
#include "math.h"

#define PI 3.14159265358979

/////////////////////////////////////////////////////////////////////////////
// CJoyin()  default constructor
//
// Function performs all the necessary initializations.
// Function MUST BE MODIFIED by the adaptor programmer. It may use HW API calls.
///////////////////////////////////////////////////////////////////////////// 
CJoyin::CJoyin(): DeviceId(0),Buffer(256)
{
//TO_DO: replace the following bogus initialization of the buffer ..
//..by the actual initialization, required by the adaptor program and the HW.
    //here we fill the buffer by the sine wave -- just for illustration.
	long size=Buffer.size();
    double scale=2*PI/size;
    NextPoint=Buffer.begin();
    for (int i=0;i<size;++i,++NextPoint)
    {
        *NextPoint=((1<<(Bits-1))-0.5)*sin(i*scale)-0.5;
    }
    NextPoint=Buffer.begin();
//END TO_DO

} // end of default constructor


/////////////////////////////////////////////////////////////////////////////
// Open()
//
// Function is called by the OpenDevice(), which is in turn called by the engine.
// CJoyin::Open() function's main goals are ..
// 1)to initialize the hardware and hardware dependent properties..
// 2)to expose pointers to the engine and the adaptor to each other..
// 3)to process the device ID, which is input by a user in the ML command line.
// The call to this function goes through the hierarchical chain: ..
//..CJoyin::Open() -> CswClockedDevice::Open() -> CmwDevice::Open()
// CmwDevice::Open() in its turn populates the pointer to the..
//..engine (CmwDevice::_engine), which allows to access all engine interfaces.
// Function MUST BE MODIFIED by the adaptor programmer.
//////////////////////////////////////////////////////////////////////////////
HRESULT CJoyin::Open(IUnknown *Interface,long ID)
{
	RETURN_HRESULT(TBaseObj::Open(Interface));
    EnableSwClocking();

//TO_DO: 1) Deal with the device ID is necessary.
//		 2) Initialize properties in accordance with the HW requirements..
//..	 3)	Initialize the hardware if necessary
	JOYINFOEX joyinfo;
	UINT wNumDevs = 1, wDeviceID;
	BOOL bDev1Attached = false, bDev2Attached = false;  
    
//	if((wNumDevs = joyGetNumDevs()) == 0)
//		return E_FAIL;		//ERR_NODRIVER;

	joyinfo.dwSize = sizeof(joyinfo);
	joyinfo.dwFlags = JOY_RETURNALL;
	int jRes1 = joyGetPosEx(JOYSTICKID1,&joyinfo);
    bDev1Attached = (jRes1 != JOYERR_UNPLUGGED); 
//    bDev2Attached = ( (wNumDevs == 2) && (joyGetPos(JOYSTICKID2,&joyinfo) !=  JOYERR_UNPLUGGED) );
	
    if(bDev1Attached || bDev2Attached)   // decide which joystick to use 
        wDeviceID = bDev1Attached ? JOYSTICKID1 : JOYSTICKID2;
	else 
        return E_FAIL;		//ERR_NODEVICE;

	m_uIdJoyDevInUse = wDeviceID;
	m_uJoyNumOfDev = wNumDevs;
		
//END TO_DO
    RETURN_HRESULT(InitHwInfo(VT_I2,Bits,4,DI_INPUT,CJoyadapt::ConstructorName,L"SwJoy"));

    return S_OK;
} // end of Open()


/////////////////////////////////////////////////////////////////////////////
// GetSingleValue()
//
// This function gets one single data point form one A/D channel, specified..
//..as a parameter.
// Function is called by GetSingleValues() of the TADDevice class, which..
// ..in turn is called by the engine as a responce to the ML user command GETSAMPLE.
// Function MUST BE MODIFIED by the adaptor programmer.
/////////////////////////////////////////////////////////////////////////////
HRESULT CJoyin::GetSingleValue(int chan,RawDataType *value)
{

//TO_DO: use HW API to elicit the data point on the specified channel..
//..and assign it to *value.
//The following code is for getting bogus samples. Remove it in the "real" adaptor.
//    *value=*NextPoint++;
//    if (NextPoint==Buffer.end())
//        NextPoint=Buffer.begin();


//	typedef struct joyinfoex_tag {     DWORD dwSize;     DWORD dwFlags; 
//    DWORD dwXpos;     DWORD dwYpos;     DWORD dwZpos;     DWORD dwRpos; 
//    DWORD dwUpos;     DWORD dwVpos;     DWORD dwButtons; 
//    DWORD dwButtonNumber;     DWORD dwPOV;     DWORD dwReserved1; 
//    DWORD dwReserved2; } JOYINFOEX; 
	
	//UINT wXpos;     UINT wYpos;     UINT wZpos; UINT wButtons;

	JOYINFOEX stJI;
	stJI.dwSize = sizeof(stJI);
	stJI.dwFlags = JOY_RETURNALL;
	if (joyGetPosEx( m_uIdJoyDevInUse, (LPJOYINFOEX) &stJI ) != JOYERR_NOERROR )
		return E_FAIL;

	switch (chan)
	{
	case 0:
		*value = (RawDataType) stJI.dwXpos - SHRT_MAX -1;
		break;
	case 1:
		*value = (RawDataType) stJI.dwYpos - SHRT_MAX -1;
		break;
	default:
		return E_FAIL;
	}

    return S_OK;
//END TO_DO

} // end of GetSingleValue()


/////////////////////////////////////////////////////////////////////////////
// InterfaceSupportsErrorInfo()
//
// Function indicates whether or not an interface supports the IErrorInfo..
//..interface. It is created by the wizard.
// Function is NOT MODIFIED by the adaptor programmer.
/////////////////////////////////////////////////////////////////////////////
STDMETHODIMP CJoyin::InterfaceSupportsErrorInfo(REFIID riid)
{
	static const IID* arr[] = 
	{
		&IID_IJoyin
	};
	for (int i=0; i < sizeof(arr) / sizeof(arr[0]); i++)
	{
		if (InlineIsEqualGUID(*arr[i],riid))
			return S_OK;
	}
	return S_FALSE;
} // end of InterfaceSupportsErrorInfo()
