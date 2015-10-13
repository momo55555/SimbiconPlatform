
// MyMFCGraphicsShaderFrameworkView.cpp : implementation of the CMyMFCGraphicsShaderFrameworkView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "MyMFCGraphicsShaderFramework.h"
#endif

#include "MyMFCGraphicsShaderFrameworkDoc.h"
#include "MyMFCGraphicsShaderFrameworkView.h"

// Temporarily loaded headers, support openGL render context, needs to be refactored later(For DX and GL contexts both)
#include <gl/glew.h>
#include <gl/wglew.h>
#include <gl/freeglut.h>

#include <windows.h>
#include <string>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;

// CMyMFCGraphicsShaderFrameworkView

IMPLEMENT_DYNCREATE(CMyMFCGraphicsShaderFrameworkView, CView)

BEGIN_MESSAGE_MAP(CMyMFCGraphicsShaderFrameworkView, CView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CMyMFCGraphicsShaderFrameworkView::OnFilePrintPreview)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_DESTROY()
	ON_WM_ERASEBKGND()
	ON_WM_TIMER()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_KEYDOWN()
	ON_WM_KEYUP()
	ON_WM_RBUTTONDOWN()
END_MESSAGE_MAP()

// CMyMFCGraphicsShaderFrameworkView construction/destruction

CMyMFCGraphicsShaderFrameworkView::CMyMFCGraphicsShaderFrameworkView()
{
	// TODO: add construction code here
}

CMyMFCGraphicsShaderFrameworkView::~CMyMFCGraphicsShaderFrameworkView()
{
}

BOOL CMyMFCGraphicsShaderFrameworkView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

    // These two styles are required
	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

	return CView::PreCreateWindow(cs);
}

// CMyMFCGraphicsShaderFrameworkView drawing

void CMyMFCGraphicsShaderFrameworkView::OnDraw(CDC* /*pDC*/)
{
	CMyMFCGraphicsShaderFrameworkDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here

    // Render callback
    pApp->Render();

    // Swap Buffers
    SwapBuffers(m_pDC->GetSafeHdc());
}


// CMyMFCGraphicsShaderFrameworkView printing


void CMyMFCGraphicsShaderFrameworkView::OnFilePrintPreview()
{
#ifndef SHARED_HANDLERS
	AFXPrintPreview(this);
#endif
}

BOOL CMyMFCGraphicsShaderFrameworkView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CMyMFCGraphicsShaderFrameworkView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CMyMFCGraphicsShaderFrameworkView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}

void CMyMFCGraphicsShaderFrameworkView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	//theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CMyMFCGraphicsShaderFrameworkView diagnostics

#ifdef _DEBUG
void CMyMFCGraphicsShaderFrameworkView::AssertValid() const
{
	CView::AssertValid();
}

void CMyMFCGraphicsShaderFrameworkView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CMyMFCGraphicsShaderFrameworkDoc* CMyMFCGraphicsShaderFrameworkView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMyMFCGraphicsShaderFrameworkDoc)));
	return (CMyMFCGraphicsShaderFrameworkDoc*)m_pDocument;
}
#endif //_DEBUG


// CMyMFCGraphicsShaderFrameworkView message handlers

int CMyMFCGraphicsShaderFrameworkView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here
	if (InitializeOpenGL())  
	{  
        // Set display refreshed interval
		SetTimer(1, 10, NULL);
        
        // Initialize GL render data
        pApp = new CBVHPlayer();
        if(!pApp->Initialize())
        {
            MessageBox(_T("pApp initialization failed"));
            return -1;
        }

		return 0;  
	}
	
    MessageBox(_T("OpenGL initialzation failed"));
	return -1;
}

void CMyMFCGraphicsShaderFrameworkView::OnDestroy()
{
	CView::OnDestroy();

	// TODO: Add your message handler code here

    //
    delete pApp;

    // Kill the timer
	KillTimer(1);

    // Destroy rendering context
	m_hRC = wglGetCurrentContext();
	if (wglMakeCurrent(NULL, NULL) == FALSE)  
	{  
		MessageBox(_T("Could not make RC non-current"));  
	}  

	if (m_hRC)
	{  
		if(wglDeleteContext(m_hRC) == FALSE)  
		{  
			MessageBox(_T("Could not delete RC"));  
		} 
	}
	else
	{
		MessageBox(_T("Find no RC"));
	}

	if (m_pDC)  
	{  
		delete m_pDC;  
	}
	m_pDC = NULL;

    pApp->ShutdownPhysics();

	exit(0);
}


BOOL CMyMFCGraphicsShaderFrameworkView::OnEraseBkgnd(CDC* pDC)
{
	// TODO: Add your message handler code here and/or call default

    // So be it
	return TRUE;

	//return CView::OnEraseBkgnd(pDC);
}

BOOL CMyMFCGraphicsShaderFrameworkView::InitializeOpenGL(void)
{
	PIXELFORMATDESCRIPTOR pfd;
	m_pDC = new CClientDC(this);
	ASSERT(m_pDC != NULL);
	
	if (!SetupPixelFormat())
	{
		MessageBox(_T("Can't set pixel formats"));
		return FALSE;
	}

	HGLRC tempOpenGLContext = wglCreateContext(m_pDC->GetSafeHdc()); // Create an OpenGL 2.1 context for our device context  
	wglMakeCurrent(m_pDC->GetSafeHdc(), tempOpenGLContext); // Make the OpenGL 2.1 context current and active 

    // init OpenGL extension wrangler
	int res;
	res = glewInit();
	if (res != GLEW_OK)
	{
		MessageBox(_T(" GLEW could not be initialized "));
		return FALSE;
	}

	GLint MajorVer;
	glGetIntegerv(GL_MAJOR_VERSION, &MajorVer);
	GLint MinorVer;
	glGetIntegerv(GL_MINOR_VERSION, &MinorVer);

	// Create higher version of OpenGL context
#ifdef PHYSX_DEBUGGING
    GLint attribs[] = {WGL_CONTEXT_MAJOR_VERSION_ARB, 3, WGL_CONTEXT_MINOR_VERSION_ARB, 0, 0};
#else
    GLint attribs[] = {WGL_CONTEXT_MAJOR_VERSION_ARB, MajorVer, WGL_CONTEXT_MINOR_VERSION_ARB, MinorVer, 0};
#endif
	if (wglewIsSupported("WGL_ARB_create_context") == 1)
	{
		m_hRC = wglCreateContextAttribsARB(::GetDC(GetSafeHwnd()), 0, attribs);

		if (m_hRC == NULL)
		{
			MessageBox(_T("HRC is NULL"));
			return FALSE;
		}

		wglMakeCurrent(NULL, NULL); // Remove the temporary context from being active  
		wglDeleteContext(tempOpenGLContext); // Delete the temporary OpenGL 2.1 context

		if (wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC) == FALSE)
		{
			MessageBox(_T("Cannot make current wgls"));
			return FALSE;
		}
	}
	else
	{
		MessageBox(_T("WGL_ARB_create_context not supported"));
		return FALSE;
	}

	return TRUE;
}

BOOL CMyMFCGraphicsShaderFrameworkView::SetupPixelFormat()
{
	static PIXELFORMATDESCRIPTOR pfd = 
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1, 
		PFD_DRAW_TO_WINDOW |
		PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		24,
		0, 0, 0, 0, 0, 0,
		0,
		0,
		0,
		0, 0, 0, 0,
		32,     
		0,
		0,
		PFD_MAIN_PLANE,
		0, 
		0, 0, 0
	};

	int pixelFormat;

	// get the closed matched pixel format
	if ((pixelFormat = ChoosePixelFormat(m_pDC->GetSafeHdc(), &pfd)) == 0)
	{
		MessageBox(_T("ChoosePixelFormat failed"));
		return FALSE;
	}

	if (SetPixelFormat(m_pDC->GetSafeHdc(), pixelFormat, &pfd) == FALSE)
	{
		MessageBox(_T("SetPixelFormat failed"));
		return FALSE;
	}
	return TRUE;
}

static int initOnSizeFlag = 0;

void CMyMFCGraphicsShaderFrameworkView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	
    m_width = cx;     
	m_height = cy;
	// avoid such circumstance that m_height equals 0  
	if(m_height == 0)  
	{  
		m_height = 1;  
	}  

#ifndef PHYSX_DEBUGGING
    if(!initOnSizeFlag)
    {
        Vector3f Pos(0.0f, 2.0f, 3.0f);
        Vector3f Target(0.0f, 0.0f, -1.0f);
        Target.Normalize();
        Vector3f Up(0.0, 1.0f, 1.0f);
        Up.Normalize();
 
        pApp->m_GameCamera = new Camera(m_width, m_height, Pos, Target, Up);
        pApp->m_persProjInfo.FOV = 60.0f;
        pApp->m_persProjInfo.Height = 1.0f;
        pApp->m_persProjInfo.Width = 1.0f;
        pApp->m_persProjInfo.zNear = 1.0f;
        pApp->m_persProjInfo.zFar = 1000.0f;     
 
        pApp->m_SkyBox = new SkyBox(pApp->m_GameCamera, pApp->m_persProjInfo);
 
        if (!pApp->m_SkyBox->Init(".",
            "../Data/Texture/Skybox/Sunny3_left.jpg",
            "../Data/Texture/Skybox/Sunny3_right.jpg",
            "../Data/Texture/Skybox/Sunny3_up.jpg",
            "../Data/Texture/Skybox/Sunny3_down.jpg",
            "../Data/Texture/Skybox/Sunny3_front.jpg",
            "../Data/Texture/Skybox/Sunny3_back.jpg")) 
        {
            MessageBox(_T("SkyBox initialization failed"));
            exit(0);
        }
 
        initOnSizeFlag = 1;
    }
 
    // set view port
    pApp->m_persProjInfo.FOV = 60.0f;
    pApp->m_persProjInfo.Height = 1.0f;
    pApp->m_persProjInfo.Width = 1.0f;
    pApp->m_persProjInfo.zNear = 1.0f;
    pApp->m_persProjInfo.zFar = 1000.0f;       
 
    pApp->m_GameCamera->SetWindowSize(m_width, m_height);
    pApp->m_SkyBox->WindowSizeChange(pApp->m_GameCamera, pApp->m_persProjInfo);
#endif

	glViewport(0, 0, m_width, m_height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (GLfloat)m_width / (GLfloat)m_height, 0.1f, 100000.0f);
    glMatrixMode(GL_MODELVIEW);
}

static int LButtonFlag = 0, RButtonFlag = 0;

void CMyMFCGraphicsShaderFrameworkView::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default

    // Deliver repaint message
    Invalidate();
	CView::OnTimer(nIDEvent);
}

void CMyMFCGraphicsShaderFrameworkView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
#ifdef PHYSX_DEBUGGING
    LButtonFlag = 1;
    pApp->gOldMouseX = point.x;
    pApp->gOldMouseY = point.y;
#endif

	CView::OnLButtonDown(nFlags, point);
}

void CMyMFCGraphicsShaderFrameworkView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
#ifdef PHYSX_DEBUGGING
    LButtonFlag = 0;
#endif

	CView::OnLButtonUp(nFlags, point);
}

void CMyMFCGraphicsShaderFrameworkView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
#ifndef PHYSX_DEBUGGING
    if(RButtonFlag)
    {
        pApp->m_GameCamera->OnMouse(point.x, point.y);
    }
#else
    if(LButtonFlag)
    {
        pApp->gCamRoateY += (point.x - pApp->gOldMouseX)/5.0f;
        pApp->gCamRoateX += (point.y - pApp->gOldMouseY)/5.0f;
    }
    if(RButtonFlag)
    {
        pApp->gCamDistance -= (point.y - pApp->gOldMouseY)/5.0f;
    }

    pApp->gOldMouseX = point.x;
    pApp->gOldMouseY = point.y;
#endif

	CView::OnMouseMove(nFlags, point);
}

void CMyMFCGraphicsShaderFrameworkView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: Add your message handler code here and/or call default
#ifndef PHYSX_DEBUGGING
    pApp->m_GameCamera->OnKeyboard(nChar);
#endif

	CView::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CMyMFCGraphicsShaderFrameworkView::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: Add your message handler code here and/or call default

	CView::OnKeyUp(nChar, nRepCnt, nFlags);
}

void CMyMFCGraphicsShaderFrameworkView::OnRButtonUp(UINT nFlags, CPoint point)
{
	//ClientToScreen(&point);
	//OnContextMenu(this, point);

#ifndef PHYSX_DEBUGGING
    RButtonFlag = 0;
    pApp->m_GameCamera->StopMousecap();
#else
    RButtonFlag = 0;
#endif

	CView::OnRButtonUp(nFlags, point);
}

void CMyMFCGraphicsShaderFrameworkView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
#ifndef PHYSX_DEBUGGING
    RButtonFlag = 1;
    pApp->m_GameCamera->BeginMouseCap();
    pApp->m_GameCamera->SetMousePos(point.x, point.y);
#else
    RButtonFlag = 1;
    pApp->gOldMouseX = point.x;
    pApp->gOldMouseY = point.y;
#endif

	CView::OnRButtonDown(nFlags, point);
}
