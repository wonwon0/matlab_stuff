// Joyadapt.h : Declaration of the CJoyadapt class
// Copyright 1998-2000 by The MathWorks, Inc.
// $Revision: 1 $  $Date: 1/23/01 3:22p $


#ifndef __JoyADAPT_H_
#define __JoyADAPT_H_

#include "resource.h"       // main symbols

/////////////////////////////////////////////////////////////////////////////
// CJoyadapt class -- implements ImwAdaptor
//
class ATL_NO_VTABLE CJoyadapt : 
	public CComObjectRootEx<CComMultiThreadModel>,
	public CComCoClass<CJoyadapt, &CLSID_Joyadapt>,
	public ImwAdaptor
{
public:
DECLARE_NOT_AGGREGATABLE(CJoyadapt)

DECLARE_PROTECT_FINAL_CONSTRUCT()
DECLARE_CLASSFACTORY_SINGLETON(CJoyadapt)
BEGIN_COM_MAP(CJoyadapt)
	COM_INTERFACE_ENTRY(ImwAdaptor)
END_COM_MAP()

BEGIN_CATEGORY_MAP(CJoyadapt)
    IMPLEMENTED_CATEGORY(CATID_ImwAdaptor)
END_CATEGORY_MAP()

//TO_DO: In this macro enter (1)name of the adaptor class,..
//..(2)program ID, (3)version independent program ID, (4)index of the name..
//..bearing resource string -- from the resource.h file. Keep flags untouched.
DECLARE_REGISTRY( CJoyadapt, _T("Joy.Joyadapt.1"), _T("Joy.Joyadapt"),
				  IDS_PROJNAME, THREADFLAGS_BOTH )
//END TO_DO

public:
	CJoyadapt();
	~CJoyadapt();
	STDMETHOD(AdaptorInfo)(IPropContainer * Container);
    STDMETHOD(OpenDevice)(REFIID riid, long nParams, VARIANT __RPC_FAR *Param,
              REFIID EngineIID, IUnknown __RPC_FAR *pIEngine,
              void __RPC_FAR *__RPC_FAR *ppIDevice);
    STDMETHOD(TranslateError)(HRESULT eCode, BSTR * retVal);

    static OLECHAR ConstructorName[100];

private:
LPWSTR StringToLower(LPCWSTR in,LPWSTR out) 
{
    while(*in)
		*out++ = towlower(*in++);
    return out;
} // end of StringToLower()

};

#endif //__JoyADAPT_H_
