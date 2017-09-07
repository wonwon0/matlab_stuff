// Joyin.h : Declaration of CJoyin class
// Copyright 1998-2000 by The MathWorks, Inc.
// $Revision: 1 $  $Date: 1/23/01 3:22p $


#ifndef __JoyIN_H_
#define __JoyIN_H_

#include "resource.h"       // main symbols


//This abstract class extends the CswClockedDevice class by a single ..
//..pure virtual function GetSingleValue() 
class ATL_NO_VTABLE CJoyInputBase: public CswClockedDevice
{
public:
    typedef short RawDataType; 
    enum BitsEnum {Bits=16}; // bits must fit in rawdatatype 
    virtual HRESULT GetSingleValue(int index,RawDataType *Value)=0;
};

/////////////////////////////////////////////////////////////////////////////
// CJoyin class declaration
//
// CJoyin is based on ImwDevice and ImwInput via chains:..
//.. ImwDevice -> CmwDevice -> CswClockedDevice -> CJoyInputBase ->..
//.. TADDevice -> CJoyin  and.. 
//.. ImwInput -> TADDevice -> CJoyin
class ATL_NO_VTABLE CJoyin : 
	public TADDevice<CJoyInputBase>, //is based on ImwDevice
	public CComCoClass<CJoyin, &CLSID_Joyin>,
	public ISupportErrorInfo,
	public IDispatchImpl<IJoyin, &IID_IJoyin, &LIBID_JOYLib>
{
    typedef TADDevice<CJoyInputBase> TBaseObj;

public:

//TO_DO: In this macro enter (1)name of the adaptor class,..
//..(2)program ID, (3)version independent program ID, (4)index of the name..
//..bearing resource string -- from the resource.h file. Keep flags untouched.
DECLARE_REGISTRY( CJoyin, _T("Joy.Joyin.1"), _T("Joy.Joyin"),
				  IDS_PROJNAME, THREADFLAGS_BOTH )
//END TO_DO

//this line is not needed if the program does not support aggregation
DECLARE_PROTECT_FINAL_CONSTRUCT()

//ATL macros internally implementing QueryInterface() for the mapped interfaces
BEGIN_COM_MAP(CJoyin)
	COM_INTERFACE_ENTRY(IJoyin)
	COM_INTERFACE_ENTRY(ImwDevice)
	COM_INTERFACE_ENTRY(ImwInput)
	COM_INTERFACE_ENTRY(IDispatch)
	COM_INTERFACE_ENTRY(ISupportErrorInfo)
END_COM_MAP()

public:
	CJoyin();
    HRESULT Open(IUnknown *Interface,long ID);
    STDMETHOD(InterfaceSupportsErrorInfo)(REFIID riid);
    HRESULT GetSingleValue(int chan,RawDataType *value);

//DeviceId data member currently is not used in the Joy program
    UINT DeviceId;

    typedef std::vector<RawDataType> BufferT;
    BufferT Buffer;
    BufferT::iterator NextPoint;

private:
	UINT m_uJoyNumOfDev;
	UINT m_uIdJoyDevInUse;
};

#endif //__JoyIN_H_
