// ���`4���؏Փ˔���T���v���v���O����
//  ����� : IKD
//   ���~����`�ǂ��ƃR���ihttp://www.h6.dion.ne.jp/~game296o/�j

#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "d3d9.lib")
#pragma comment(lib, "d3dx9.lib")

#include <windows.h>
#include <tchar.h>
#include <d3d9.h>
#include <d3dx9.h>
#include "CollisionAPI.h"	// �Փ˔���API
#include "ColTrees.h"	// ���`4���؃w�b�_�[
#include <time.h>
#include "FPSCounter.h"


using namespace std;
using namespace IKD;


_TCHAR gName[100] = _T("3D�I�u�W�F�N�g�`��T���v���v���O����");


///////////////////////////////////////////////
// �O���[�o���ϐ��Q�i���l��ς��ėV�ׂ܂��j
////////////////
float g_Circle_Ref = 0.87f;	// �����m�̔����W��
float Wall_Ref = 0.85f;		// ���ƕǂ̔����W��
const int g_CircleNum = 650;	// ��������~�̐�
int g_PartitionLebel = 6;		// ��ԕ������x��
float g_Gravity = 9.8f;			// �d��

ID3DXFont *font = 0;			// �t�H���g


// �~�\����
struct CIRCLE
{
	DWORD ID;
	float x, y;				// �~�̈ʒu
	float Pre_x, Pre_y;		// 1�O�̉~�̈ʒu
	float vx, vy;			// ���x�x�N�g��
	float ax, ay;			// �����x�x�N�g��
	float r;				// ���a
	float w;				// ����
	float scale;			// �X�P�[��
	IDirect3DTexture9 *pTex;	// �`��e�N�X�`��

	CIRCLE()
	{
		x = y = vx = vy = ax = ay = Pre_x = Pre_y = 0.0f;
		r = 1.0f;
		w = 1.0f;
		scale = 1.0f;
		pTex = NULL;
	}
};


// �ǂƋ��̔��˃x�N�g�����v�Z
void GetRefrectVelo(D3DXVECTOR3 *pOut, D3DXVECTOR3 &N, D3DXVECTOR3 &V, float e)
{
	D3DXVec3Normalize(&N,&N);
	*pOut = V - (1+e)*D3DXVec3Dot(&N,&V)*N;
}


// �ǂƂ̔��ˌ�̈ʒu���Z�o
void GetRelectedPos( float Res_time, CIRCLE &circle, D3DXVECTOR3 &RefV )
{
	// �Փˈʒu
	// 0.99�͕ǂɂ߂荞�܂Ȃ����߂̕␳
	circle.x = circle.Pre_x + circle.vx * (1-Res_time)*0.99f;
	circle.y = circle.Pre_y + circle.vy * (1-Res_time)*0.99f;
	// ���˃x�N�g��
	circle.vx = RefV.x;
	circle.vy = RefV.y;
	// �ʒu��␳
	circle.x += circle.vx * Res_time;
	circle.y += circle.vy * Res_time;
}


// ���̉~�̈ʒu���Z�o
void GetNextCirclePos( CIRCLE &circle )
{
	D3DXVECTOR3 RefV;	// ���ˌ�̑��x�x�N�g��
	D3DXVECTOR3 ColliPos;	// �Փˈʒu
	float Res_time = 0.0f;	// �Փˌ�̈ړ��\����

	// �d�͂��|���ė��Ƃ�
	circle.vy += g_Gravity/60;	// 1�t���[����9.8/60(m/s)����

	// ���̑��x�ňʒu���X�V
	circle.Pre_x = circle.x;	// �O�̈ʒu��ۑ�
	circle.Pre_y = circle.y;
	circle.x += circle.vx;		// �ʒu�X�V
	circle.y += circle.vy;

	// �ǂƂ̏Փ˂��`�F�b�N
	// ����
	if(circle.x<0){
		// ���ˌ�̑��x�x�N�g�����擾
		GetRefrectVelo( &RefV, D3DXVECTOR3(1,0,0), D3DXVECTOR3(circle.vx, circle.vy, 0), Wall_Ref);
		// �c�莞�ԎZ�o
		Res_time = circle.x / circle.vx;
		// ���ˌ�̈ʒu���Z�o
		GetRelectedPos(	Res_time, circle, RefV );
	}
	// �E��
	else if(circle.x>640){
		GetRefrectVelo( &RefV, D3DXVECTOR3(-1,0,0), D3DXVECTOR3(circle.vx, circle.vy, 0), Wall_Ref);
		Res_time = (circle.x-640) / circle.vx;
		GetRelectedPos(	Res_time, circle, RefV );
	}
	// ����
	else if(circle.y>480){
		GetRefrectVelo( &RefV, D3DXVECTOR3(0,-1,0), D3DXVECTOR3(circle.vx, circle.vy, 0), Wall_Ref);
		Res_time = (circle.y-480) / circle.vy;
		GetRelectedPos(	Res_time, circle, RefV );
	}
}


// 2�~�Փˏ���
void CircleColProc( CIRCLE *c1, CIRCLE *c2 )
{
	float t=0;
	D3DXVECTOR3 C1ColPos, C2ColPos, C1Velo, C2Velo;

	// �Փ˂��Ă���2�~�̏Փˈʒu�����o
	if(!CalcParticleCollision(c1->r, c2->r,
		&D3DXVECTOR3(c1->Pre_x, c1->Pre_y,0),
		&D3DXVECTOR3(c1->x, c1->y, 0),
		&D3DXVECTOR3(c2->Pre_x, c2->Pre_y, 0),
		&D3DXVECTOR3(c2->x, c2->y, 0),
		&t,
		&C1ColPos,
		&C2ColPos))
		return;	// �Փ˂��Ă��Ȃ��悤�ł�

	// �Փˈʒu��O�ʒu�Ƃ��ĕۑ�
	c1->x = C1ColPos.x;
	c1->y = C1ColPos.y; 
	c2->x = C2ColPos.x;
	c2->y = C2ColPos.y; 
	c1->Pre_x = C1ColPos.x;
	c1->Pre_y = C1ColPos.y; 
	c2->Pre_x = C2ColPos.x;
	c2->Pre_y = C2ColPos.y; 

	// �Փˌ�̑��x���Z�o
	if(!CalcParticleColliAfterPos(
		&C1ColPos, &D3DXVECTOR3(c1->vx, c1->vy, 0),
		&C2ColPos, &D3DXVECTOR3(c2->vx, c2->vy, 0),
		c1->w, c2->w,
		g_Circle_Ref, g_Circle_Ref,		// ���̔����W��
		t,
		&C1ColPos, &C1Velo,
		&C2ColPos, &C2Velo))
		return; // �������s�����悤�ł�

	// �Փˌ�ʒu�Ɉړ�
	c1->vx = C1Velo.x;
	c1->vy = C1Velo.y;
	c2->vx = C2Velo.x;
	c2->vy = C2Velo.y;
	c1->x += c1->vx;
	c1->y += c1->vy;
	c2->x += c2->vx;
	c2->y += c2->vy;
}

// �o�͗p�t�H���g�쐬
void createFont( IDirect3DDevice9* dev ) {
	D3DXCreateFont( dev, 16, 8, 0, 1, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
		DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, _T("�l�r �o�S�V�b�N"), &font );
}

// �t�H���g�����`��
void drawText( TCHAR* text, int left, int top ) {
	RECT R = { left, top, left + 300, top + 20 };
	font->DrawText( 0, text, -1, &R, DT_LEFT, 0xffffffff );
}

// �E�B���h�E�v���V�[�W��
LRESULT CALLBACK WndProc(HWND hWnd, UINT mes, WPARAM wParam, LPARAM lParam){
	switch(mes){
		case WM_DESTROY:
			PostQuitMessage(0);
			return 0L;
		case WM_CLOSE:
			DestroyWindow(hWnd);
			break;
		case WM_CHAR:
			if((TCHAR)wParam == VK_ESCAPE)
				PostQuitMessage(0);
			return 0L;
		default:
			return DefWindowProc(hWnd, mes, wParam, lParam);
	}
	return 0L;
}


// ���C���֐�
int APIENTRY _tWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPTSTR lpCmdLine, int nCmdShow)
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);	// ���������[�N�`�F�b�N
	srand((unsigned int)time(NULL));	// �����ď�����

	int ff = sizeof(CCell<CIRCLE>);

	// �A�v���P�[�V�����̏�����
	MSG msg; HWND hWnd;
	WNDCLASSEX wcex ={sizeof(WNDCLASSEX), CS_HREDRAW | CS_VREDRAW, WndProc, 0, 0, hInstance, NULL, NULL,
									(HBRUSH)(COLOR_WINDOW+1), NULL, (_TCHAR*)gName, NULL};
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW); 
	if(!RegisterClassEx(&wcex))
		return 0;

	DWORD WndStyle = WS_OVERLAPPEDWINDOW & ~(WS_MAXIMIZEBOX | WS_SIZEBOX);
	RECT WndRect={0, 0, 640, 480};
	AdjustWindowRect( &WndRect, WndStyle, false );

	if(!(hWnd = CreateWindow( gName, gName, WndStyle, CW_USEDEFAULT, 0,
	   WndRect.right-WndRect.left, WndRect.bottom-WndRect.top, NULL, NULL, hInstance, NULL)))
		return 0;

	// Direct3D�̏�����
	LPDIRECT3D9 g_pD3D;
	LPDIRECT3DDEVICE9 g_pD3DDev;
	if( !(g_pD3D = Direct3DCreate9( D3D_SDK_VERSION )) ) return 0;

	bool WindowMode = true;	// �E�B���h�E���[�h�ƃt���X�N���[�����[�h�̐؂�ւ�
	D3DPRESENT_PARAMETERS d3dpp = {640,480,D3DFMT_A8R8G8B8,0,D3DMULTISAMPLE_NONE,0,D3DSWAPEFFECT_DISCARD,hWnd,WindowMode,FALSE,D3DFMT_D24S8,0,D3DPRESENT_RATE_DEFAULT, /*D3DPRESENT_INTERVAL_IMMEDIATE*/ D3DPRESENT_INTERVAL_DEFAULT }; 

	if( FAILED( g_pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &d3dpp, &g_pD3DDev ) ) )
	if( FAILED( g_pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &g_pD3DDev ) ) )
	{
		MessageBox(NULL,_T("DirectX�̏������Ɏ��s���܂���"),NULL,NULL);
		g_pD3D->Release();
		return 0;
	}

	// �o�͗p�t�H���g�쐬
	createFont( g_pD3DDev );

	/////////////////////////////
	// �~�I�u�W�F�N�g�̏�����
	ID3DXSprite *pCircleSp, *pColSp;	// �X�v���C�g
	D3DXCreateSprite(g_pD3DDev, &pCircleSp);
	D3DXCreateSprite(g_pD3DDev, &pColSp);
	IDirect3DTexture9 *pCircleTex, *pColTex;	// �e�N�X�`��
	if(FAILED(D3DXCreateTextureFromFile(g_pD3DDev, _T("Circle.png"), &pCircleTex))){
		MessageBox(NULL,_T("Circle.png�t�@�C��������܂���"),NULL,NULL);
		return 0;
	}
	if(FAILED(D3DXCreateTextureFromFile(g_pD3DDev, _T("Circle(Col).png"), &pColTex))){
		MessageBox(NULL,_T("Circle(Col).png�t�@�C��������܂���"),NULL,NULL);
		return 0;
	}
	CIRCLE CAry[ g_CircleNum ];
	sp<OBJECT_FOR_TREE<CIRCLE> > spOFTAry[g_CircleNum];	// �~�I�u�W�F�N�g����OFT�I�u�W�F�N�g

	//////////////////////////////////////////
	// �~�I�u�W�F�N�g�̏����ʒu�E���x�̐ݒ�
	//  �V�ԃ|�C���g�ł�(^-^)
	//////////
	int cn;
	for(cn=0;cn<g_CircleNum;cn++)	// g_CircleNum�����~�𐶐�
	{
		CAry[cn].ID = cn;
		CAry[cn].r = 2 + 3*(float)rand()/RAND_MAX;	// �~�̔��a(2�`5�܂Ń����_��)
		CAry[cn].x = (float)cn/g_CircleNum*120+30*(float)rand()/RAND_MAX;		// �ג��������ʒu
		CAry[cn].y = -400 + 700*(float)cn/g_CircleNum;	// ���\�����Ƃ��납�痎�Ƃ��܂�
		CAry[cn].vx = 0.5;		// �����i�K���j
		CAry[cn].vy = 0;
		CAry[cn].scale = CAry[cn].r/32.0f;				// �摜��64�~64�Ȃ̂ŃX�P�[���l�͂����Ȃ��ł��I
		CAry[cn].w = CAry[cn].r*CAry[cn].r*CAry[cn].r;	// ���ʂ͔��a��3��ɔ��Ƃ��܂�
		// OFT�ɓo�^
		OBJECT_FOR_TREE<CIRCLE> *p = new OBJECT_FOR_TREE<CIRCLE>( cn );
		p->m_pObject = &CAry[cn];	// �o�^
		spOFTAry[cn].SetPtr(p);
	}


	///////////////////////////////////
	// ���`4���؃}�l�[�W��
	//  ��Ԕ͈͂�X=-60�`720; Y=-1200�`520�ɐݒ�
	//  �~����яo���Ȃ��͈͂��w�肷��Ηǂ��̂�
	//  �A�o�E�g�ł�
	CLiner4TreeManager<CIRCLE> LTree;
	if(!LTree.Init(g_PartitionLebel, -60.0f, -1200.0f, 720.0f, 520.0f))
	{
		MessageBox(NULL,_T("���`4���؋�Ԃ̏������Ɏ��s���܂����B"),NULL,NULL);
		return 0;
	}

	// �E�B���h�E�\��
	ShowWindow( hWnd, SW_SHOW );

	// ���[�v���ꎞ�ϐ�
	DWORD ColNum;				// �Փ˔����
	CollisionList<CIRCLE>* ColVect;	// �ՓˑΏۃ��X�g
	TCHAR pText[64];
	int cnt=0;
	HDC hDC = GetDC(hWnd);
	CFPSCounter FPSCounter;
	float FPS = 0.0;

	do{
		cnt++;
		if( PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) ){
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else 
		{
			g_pD3DDev->Clear( 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(20,40,50), 1.0f, 0 );

			// �~�̈ʒu���X�V���ăc���[�ɍēo�^
			static bool change = true;
			for( cn = 0; cn < g_CircleNum; cn++ )
			{
				int i = cn;
				// ���e�X�g�̂��߂Ɉ�E�ēo�^�̏��Ԃ�ς��Ă��܂�
				if ( change == true ) {
					i = g_CircleNum - cn - 1;
				}
				CIRCLE *pTmp = spOFTAry[i]->m_pObject;
				pTmp->pTex = pCircleTex;	// �e�N�X�`��������
				GetNextCirclePos( *pTmp );	// ���̈ړ��ʒu��������
				spOFTAry[i]->Remove();		// ��x���X�g����O���
				// �ēo�^
				LTree.Regist( pTmp->x-pTmp->r, pTmp->y-pTmp->r, pTmp->x+pTmp->r, pTmp->y+pTmp->r, spOFTAry[i].GetPtr() );
			}
			change = !change;

			// �ՓˑΉ����X�g���擾
			ColNum = LTree.GetAllCollisionList( &ColVect );

			// �Փ˔���
			DWORD c;
			ColNum/=2;	// 2�Ŋ���̂̓y�A�ɂȂ��Ă���̂�
			CIRCLE** pRoot = ColVect->getRootPtr();
			for(c=0; c<ColNum; c++){
				float r2 = (pRoot[c*2]->r+pRoot[c*2+1]->r)*(pRoot[c*2]->r+pRoot[c*2+1]->r);
				float x = (pRoot[c*2]->x-pRoot[c*2+1]->x);
				float y = (pRoot[c*2]->y-pRoot[c*2+1]->y);
				if(r2 >= x*x + y*y )
				{
					// �Ԃ����������m�̃e�N�X�`�����I�����W�ɕύX
					pRoot[c*2]->pTex = pColTex;
					pRoot[c*2+1]->pTex = pColTex;

					// 2�~�Փˏ���������
					CircleColProc( pRoot[c*2], pRoot[c*2+1] );
				}
			}

			g_pD3DDev->BeginScene();

			// �`��
			D3DXMATRIX CircleMat;
			pCircleSp->Begin(D3DXSPRITE_ALPHABLEND);
			for(cn=0; cn<g_CircleNum; cn++){
				D3DXMatrixIdentity(&CircleMat);
				CircleMat._11 = CAry[cn].scale;
				CircleMat._22 = CAry[cn].scale;
				CircleMat._41 = CAry[cn].x;
				CircleMat._42 = CAry[cn].y;
				pCircleSp->SetTransform(&CircleMat);
				pCircleSp->Draw(CAry[cn].pTex,NULL,&D3DXVECTOR3(32,32,0),NULL,0xffffffff);
			}
			pCircleSp->End();

			// �c���[���\��
			_stprintf_s( pText, _T("Circle Number = %d"), g_CircleNum );
			drawText( pText, 0, 22 );
			_stprintf_s(pText, _T("Collision Check Number = %d"), ColNum/2);
			drawText( pText, 0, 44 );
			_stprintf_s(pText, _T("Optimaization = %2.5f ��"), 100.0*(float)(ColNum/2)/(g_CircleNum/2*(g_CircleNum+1)));
			drawText( pText, 0, 66 );
			_stprintf_s(pText, _T("Counter = %d"), cnt);
			drawText( pText, 0, 88 );
			_stprintf_s( pText, _T("FPS = %f"), FPS );
			drawText( pText, 0, 108 );

			FPS = (float)FPSCounter.GetFPS();

			g_pD3DDev->EndScene();
			g_pD3DDev->Present( NULL, NULL, NULL, NULL );
		}

	}while(msg.message != WM_QUIT);

	ReleaseDC(hWnd, hDC);
	g_pD3DDev->Release();
	g_pD3D->Release();

   return 0;
}