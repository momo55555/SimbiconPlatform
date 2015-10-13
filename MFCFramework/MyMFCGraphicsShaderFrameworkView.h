
// MyMFCGraphicsShaderFrameworkView.h : interface of the CMyMFCGraphicsShaderFrameworkView class
//

#pragma once

#include "BVHPlayer.h"

#define _OPENGL_RENDER

class CMyMFCGraphicsShaderFrameworkView : public CView
{
protected: // create from serialization only
	CMyMFCGraphicsShaderFrameworkView();
	DECLARE_DYNCREATE(CMyMFCGraphicsShaderFrameworkView)

// Attributes
public:
	CMyMFCGraphicsShaderFrameworkDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// Implementation
public:
	virtual ~CMyMFCGraphicsShaderFrameworkView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()

// Override message callbacks
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnDestroy();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
    afx_msg void OnMouseMove(UINT nFlags, CPoint point);
    afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
    afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
    afx_msg void OnRButtonDown(UINT nFlags, CPoint point);

// User defined functions and attributes
public:
	BOOL InitializeOpenGL();    //initialize OpenGL
	BOOL SetupPixelFormat();    //SetupPixelFormat

	HGLRC m_hRC;    //Rendering Context
    CClientDC* m_pDC;   //Device Context

private:
    int m_width;
    int m_height;
    CBVHPlayer* pApp;
};

#ifndef _DEBUG  // debug version in MyMFCGraphicsShaderFrameworkView.cpp
inline CMyMFCGraphicsShaderFrameworkDoc* CMyMFCGraphicsShaderFrameworkView::GetDocument() const
   { return reinterpret_cast<CMyMFCGraphicsShaderFrameworkDoc*>(m_pDocument); }
#endif

