/* this ALWAYS GENERATED file contains the definitions for the interfaces */


/* File created by MIDL compiler version 5.01.0164 */
/* at Wed Nov 01 15:38:07 2000
 */
/* Compiler settings for D:\Work\daqadaptor\JoyStick\Joy.idl:
    Oicf (OptLev=i2), W1, Zp8, env=Win32, ms_ext, c_ext
    error checks: allocation ref bounds_check enum stub_data 
*/
//@@MIDL_FILE_HEADING(  )


/* verify that the <rpcndr.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCNDR_H_VERSION__
#define __REQUIRED_RPCNDR_H_VERSION__ 440
#endif

#include "rpc.h"
#include "rpcndr.h"

#ifndef __RPCNDR_H_VERSION__
#error this stub requires an updated version of <rpcndr.h>
#endif // __RPCNDR_H_VERSION__

#ifndef COM_NO_WINDOWS_H
#include "windows.h"
#include "ole2.h"
#endif /*COM_NO_WINDOWS_H*/

#ifndef __Joy_h__
#define __Joy_h__

#ifdef __cplusplus
extern "C"{
#endif 

/* Forward Declarations */ 

#ifndef __IJoyin_FWD_DEFINED__
#define __IJoyin_FWD_DEFINED__
typedef interface IJoyin IJoyin;
#endif 	/* __IJoyin_FWD_DEFINED__ */


#ifndef __Joyadapt_FWD_DEFINED__
#define __Joyadapt_FWD_DEFINED__

#ifdef __cplusplus
typedef class Joyadapt Joyadapt;
#else
typedef struct Joyadapt Joyadapt;
#endif /* __cplusplus */

#endif 	/* __Joyadapt_FWD_DEFINED__ */


#ifndef __Joyin_FWD_DEFINED__
#define __Joyin_FWD_DEFINED__

#ifdef __cplusplus
typedef class Joyin Joyin;
#else
typedef struct Joyin Joyin;
#endif /* __cplusplus */

#endif 	/* __Joyin_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"
#include "daqmex.h"

void __RPC_FAR * __RPC_USER MIDL_user_allocate(size_t);
void __RPC_USER MIDL_user_free( void __RPC_FAR * ); 

#ifndef __IJoyin_INTERFACE_DEFINED__
#define __IJoyin_INTERFACE_DEFINED__

/* interface IJoyin */
/* [unique][helpstring][dual][uuid][object] */ 


EXTERN_C const IID IID_IJoyin;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("61ABE1E1-AEAD-11d4-BAAF-00D0B7BAD1D3")
    IJoyin : public IDispatch
    {
    public:
    };
    
#else 	/* C style interface */

    typedef struct IJoyinVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE __RPC_FAR *QueryInterface )( 
            IJoyin __RPC_FAR * This,
            /* [in] */ REFIID riid,
            /* [iid_is][out] */ void __RPC_FAR *__RPC_FAR *ppvObject);
        
        ULONG ( STDMETHODCALLTYPE __RPC_FAR *AddRef )( 
            IJoyin __RPC_FAR * This);
        
        ULONG ( STDMETHODCALLTYPE __RPC_FAR *Release )( 
            IJoyin __RPC_FAR * This);
        
        HRESULT ( STDMETHODCALLTYPE __RPC_FAR *GetTypeInfoCount )( 
            IJoyin __RPC_FAR * This,
            /* [out] */ UINT __RPC_FAR *pctinfo);
        
        HRESULT ( STDMETHODCALLTYPE __RPC_FAR *GetTypeInfo )( 
            IJoyin __RPC_FAR * This,
            /* [in] */ UINT iTInfo,
            /* [in] */ LCID lcid,
            /* [out] */ ITypeInfo __RPC_FAR *__RPC_FAR *ppTInfo);
        
        HRESULT ( STDMETHODCALLTYPE __RPC_FAR *GetIDsOfNames )( 
            IJoyin __RPC_FAR * This,
            /* [in] */ REFIID riid,
            /* [size_is][in] */ LPOLESTR __RPC_FAR *rgszNames,
            /* [in] */ UINT cNames,
            /* [in] */ LCID lcid,
            /* [size_is][out] */ DISPID __RPC_FAR *rgDispId);
        
        /* [local] */ HRESULT ( STDMETHODCALLTYPE __RPC_FAR *Invoke )( 
            IJoyin __RPC_FAR * This,
            /* [in] */ DISPID dispIdMember,
            /* [in] */ REFIID riid,
            /* [in] */ LCID lcid,
            /* [in] */ WORD wFlags,
            /* [out][in] */ DISPPARAMS __RPC_FAR *pDispParams,
            /* [out] */ VARIANT __RPC_FAR *pVarResult,
            /* [out] */ EXCEPINFO __RPC_FAR *pExcepInfo,
            /* [out] */ UINT __RPC_FAR *puArgErr);
        
        END_INTERFACE
    } IJoyinVtbl;

    interface IJoyin
    {
        CONST_VTBL struct IJoyinVtbl __RPC_FAR *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IJoyin_QueryInterface(This,riid,ppvObject)	\
    (This)->lpVtbl -> QueryInterface(This,riid,ppvObject)

#define IJoyin_AddRef(This)	\
    (This)->lpVtbl -> AddRef(This)

#define IJoyin_Release(This)	\
    (This)->lpVtbl -> Release(This)


#define IJoyin_GetTypeInfoCount(This,pctinfo)	\
    (This)->lpVtbl -> GetTypeInfoCount(This,pctinfo)

#define IJoyin_GetTypeInfo(This,iTInfo,lcid,ppTInfo)	\
    (This)->lpVtbl -> GetTypeInfo(This,iTInfo,lcid,ppTInfo)

#define IJoyin_GetIDsOfNames(This,riid,rgszNames,cNames,lcid,rgDispId)	\
    (This)->lpVtbl -> GetIDsOfNames(This,riid,rgszNames,cNames,lcid,rgDispId)

#define IJoyin_Invoke(This,dispIdMember,riid,lcid,wFlags,pDispParams,pVarResult,pExcepInfo,puArgErr)	\
    (This)->lpVtbl -> Invoke(This,dispIdMember,riid,lcid,wFlags,pDispParams,pVarResult,pExcepInfo,puArgErr)


#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IJoyin_INTERFACE_DEFINED__ */



#ifndef __JOYLib_LIBRARY_DEFINED__
#define __JOYLib_LIBRARY_DEFINED__

/* library JOYLib */
/* [helpstring][version][uuid] */ 


EXTERN_C const IID LIBID_JOYLib;

EXTERN_C const CLSID CLSID_Joyadapt;

#ifdef __cplusplus

class DECLSPEC_UUID("61ABE1E3-AEAD-11d4-BAAF-00D0B7BAD1D3")
Joyadapt;
#endif

EXTERN_C const CLSID CLSID_Joyin;

#ifdef __cplusplus

class DECLSPEC_UUID("61ABE1E4-AEAD-11d4-BAAF-00D0B7BAD1D3")
Joyin;
#endif
#endif /* __JOYLib_LIBRARY_DEFINED__ */

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif
