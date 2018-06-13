// 線形4分木衝突判定サンプルプログラム
//  製作者 : IKD
//   ○×つくろ～どっとコム（http://www.h6.dion.ne.jp/~game296o/）

#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "d3d9.lib")
#pragma comment(lib, "d3dx9.lib")

#include <windows.h>
#include <tchar.h>
#include <d3d9.h>
#include <d3dx9.h>
#include "CollisionAPI.h"	// 衝突判定API
#include "ColTrees.h"	// 線形4分木ヘッダー
#include <time.h>
#include "FPSCounter.h"


using namespace std;
using namespace IKD;


_TCHAR gName[100] = _T("3Dオブジェクト描画サンプルプログラム");


///////////////////////////////////////////////
// グローバル変数群（数値を変えて遊べます）
////////////////
float g_Circle_Ref = 0.87f;	// 球同士の反発係数
float Wall_Ref = 0.85f;		// 球と壁の反発係数
const int g_CircleNum = 650;	// 生成する円の数
int g_PartitionLebel = 6;		// 空間分割レベル
float g_Gravity = 9.8f;			// 重力

ID3DXFont *font = 0;			// フォント


// 円構造体
struct CIRCLE
{
	DWORD ID;
	float x, y;				// 円の位置
	float Pre_x, Pre_y;		// 1つ前の円の位置
	float vx, vy;			// 速度ベクトル
	float ax, ay;			// 加速度ベクトル
	float r;				// 半径
	float w;				// 質量
	float scale;			// スケール
	IDirect3DTexture9 *pTex;	// 描画テクスチャ

	CIRCLE()
	{
		x = y = vx = vy = ax = ay = Pre_x = Pre_y = 0.0f;
		r = 1.0f;
		w = 1.0f;
		scale = 1.0f;
		pTex = NULL;
	}
};


// 壁と球の反射ベクトルを計算
void GetRefrectVelo(D3DXVECTOR3 *pOut, D3DXVECTOR3 &N, D3DXVECTOR3 &V, float e)
{
	D3DXVec3Normalize(&N,&N);
	*pOut = V - (1+e)*D3DXVec3Dot(&N,&V)*N;
}


// 壁との反射後の位置を算出
void GetRelectedPos( float Res_time, CIRCLE &circle, D3DXVECTOR3 &RefV )
{
	// 衝突位置
	// 0.99は壁にめり込まないための補正
	circle.x = circle.Pre_x + circle.vx * (1-Res_time)*0.99f;
	circle.y = circle.Pre_y + circle.vy * (1-Res_time)*0.99f;
	// 反射ベクトル
	circle.vx = RefV.x;
	circle.vy = RefV.y;
	// 位置を補正
	circle.x += circle.vx * Res_time;
	circle.y += circle.vy * Res_time;
}


// 次の円の位置を算出
void GetNextCirclePos( CIRCLE &circle )
{
	D3DXVECTOR3 RefV;	// 反射後の速度ベクトル
	D3DXVECTOR3 ColliPos;	// 衝突位置
	float Res_time = 0.0f;	// 衝突後の移動可能時間

	// 重力を掛けて落とす
	circle.vy += g_Gravity/60;	// 1フレームで9.8/60(m/s)加速

	// 今の速度で位置を更新
	circle.Pre_x = circle.x;	// 前の位置を保存
	circle.Pre_y = circle.y;
	circle.x += circle.vx;		// 位置更新
	circle.y += circle.vy;

	// 壁との衝突をチェック
	// 左壁
	if(circle.x<0){
		// 反射後の速度ベクトルを取得
		GetRefrectVelo( &RefV, D3DXVECTOR3(1,0,0), D3DXVECTOR3(circle.vx, circle.vy, 0), Wall_Ref);
		// 残り時間算出
		Res_time = circle.x / circle.vx;
		// 反射後の位置を算出
		GetRelectedPos(	Res_time, circle, RefV );
	}
	// 右壁
	else if(circle.x>640){
		GetRefrectVelo( &RefV, D3DXVECTOR3(-1,0,0), D3DXVECTOR3(circle.vx, circle.vy, 0), Wall_Ref);
		Res_time = (circle.x-640) / circle.vx;
		GetRelectedPos(	Res_time, circle, RefV );
	}
	// 下壁
	else if(circle.y>480){
		GetRefrectVelo( &RefV, D3DXVECTOR3(0,-1,0), D3DXVECTOR3(circle.vx, circle.vy, 0), Wall_Ref);
		Res_time = (circle.y-480) / circle.vy;
		GetRelectedPos(	Res_time, circle, RefV );
	}
}


// 2円衝突処理
void CircleColProc( CIRCLE *c1, CIRCLE *c2 )
{
	float t=0;
	D3DXVECTOR3 C1ColPos, C2ColPos, C1Velo, C2Velo;

	// 衝突している2円の衝突位置を検出
	if(!CalcParticleCollision(c1->r, c2->r,
		&D3DXVECTOR3(c1->Pre_x, c1->Pre_y,0),
		&D3DXVECTOR3(c1->x, c1->y, 0),
		&D3DXVECTOR3(c2->Pre_x, c2->Pre_y, 0),
		&D3DXVECTOR3(c2->x, c2->y, 0),
		&t,
		&C1ColPos,
		&C2ColPos))
		return;	// 衝突していないようです

	// 衝突位置を前位置として保存
	c1->x = C1ColPos.x;
	c1->y = C1ColPos.y; 
	c2->x = C2ColPos.x;
	c2->y = C2ColPos.y; 
	c1->Pre_x = C1ColPos.x;
	c1->Pre_y = C1ColPos.y; 
	c2->Pre_x = C2ColPos.x;
	c2->Pre_y = C2ColPos.y; 

	// 衝突後の速度を算出
	if(!CalcParticleColliAfterPos(
		&C1ColPos, &D3DXVECTOR3(c1->vx, c1->vy, 0),
		&C2ColPos, &D3DXVECTOR3(c2->vx, c2->vy, 0),
		c1->w, c2->w,
		g_Circle_Ref, g_Circle_Ref,		// 球の反発係数
		t,
		&C1ColPos, &C1Velo,
		&C2ColPos, &C2Velo))
		return; // 何か失敗したようです

	// 衝突後位置に移動
	c1->vx = C1Velo.x;
	c1->vy = C1Velo.y;
	c2->vx = C2Velo.x;
	c2->vy = C2Velo.y;
	c1->x += c1->vx;
	c1->y += c1->vy;
	c2->x += c2->vx;
	c2->y += c2->vy;
}

// 出力用フォント作成
void createFont( IDirect3DDevice9* dev ) {
	D3DXCreateFont( dev, 16, 8, 0, 1, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
		DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, _T("ＭＳ Ｐゴシック"), &font );
}

// フォント文字描画
void drawText( TCHAR* text, int left, int top ) {
	RECT R = { left, top, left + 300, top + 20 };
	font->DrawText( 0, text, -1, &R, DT_LEFT, 0xffffffff );
}

// ウィンドウプロシージャ
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


// メイン関数
int APIENTRY _tWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPTSTR lpCmdLine, int nCmdShow)
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);	// メモリリークチェック
	srand((unsigned int)time(NULL));	// 乱数再初期化

	int ff = sizeof(CCell<CIRCLE>);

	// アプリケーションの初期化
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

	// Direct3Dの初期化
	LPDIRECT3D9 g_pD3D;
	LPDIRECT3DDEVICE9 g_pD3DDev;
	if( !(g_pD3D = Direct3DCreate9( D3D_SDK_VERSION )) ) return 0;

	bool WindowMode = true;	// ウィンドウモードとフルスクリーンモードの切り替え
	D3DPRESENT_PARAMETERS d3dpp = {640,480,D3DFMT_A8R8G8B8,0,D3DMULTISAMPLE_NONE,0,D3DSWAPEFFECT_DISCARD,hWnd,WindowMode,FALSE,D3DFMT_D24S8,0,D3DPRESENT_RATE_DEFAULT, /*D3DPRESENT_INTERVAL_IMMEDIATE*/ D3DPRESENT_INTERVAL_DEFAULT }; 

	if( FAILED( g_pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &d3dpp, &g_pD3DDev ) ) )
	if( FAILED( g_pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &g_pD3DDev ) ) )
	{
		MessageBox(NULL,_T("DirectXの初期化に失敗しました"),NULL,NULL);
		g_pD3D->Release();
		return 0;
	}

	// 出力用フォント作成
	createFont( g_pD3DDev );

	/////////////////////////////
	// 円オブジェクトの初期化
	ID3DXSprite *pCircleSp, *pColSp;	// スプライト
	D3DXCreateSprite(g_pD3DDev, &pCircleSp);
	D3DXCreateSprite(g_pD3DDev, &pColSp);
	IDirect3DTexture9 *pCircleTex, *pColTex;	// テクスチャ
	if(FAILED(D3DXCreateTextureFromFile(g_pD3DDev, _T("Circle.png"), &pCircleTex))){
		MessageBox(NULL,_T("Circle.pngファイルがありません"),NULL,NULL);
		return 0;
	}
	if(FAILED(D3DXCreateTextureFromFile(g_pD3DDev, _T("Circle(Col).png"), &pColTex))){
		MessageBox(NULL,_T("Circle(Col).pngファイルがありません"),NULL,NULL);
		return 0;
	}
	CIRCLE CAry[ g_CircleNum ];
	sp<OBJECT_FOR_TREE<CIRCLE> > spOFTAry[g_CircleNum];	// 円オブジェクトを包むOFTオブジェクト

	//////////////////////////////////////////
	// 円オブジェクトの初期位置・速度の設定
	//  遊ぶポイントです(^-^)
	//////////
	int cn;
	for(cn=0;cn<g_CircleNum;cn++)	// g_CircleNumだけ円を生成
	{
		CAry[cn].ID = cn;
		CAry[cn].r = 2 + 3*(float)rand()/RAND_MAX;	// 円の半径(2～5までランダム)
		CAry[cn].x = (float)cn/g_CircleNum*120+30*(float)rand()/RAND_MAX;		// 細長い初期位置
		CAry[cn].y = -400 + 700*(float)cn/g_CircleNum;	// 結構高いところから落とします
		CAry[cn].vx = 0.5;		// 初速（適当）
		CAry[cn].vy = 0;
		CAry[cn].scale = CAry[cn].r/32.0f;				// 画像が64×64なのでスケール値はこうなるんです！
		CAry[cn].w = CAry[cn].r*CAry[cn].r*CAry[cn].r;	// 質量は半径の3乗に比例とします
		// OFTに登録
		OBJECT_FOR_TREE<CIRCLE> *p = new OBJECT_FOR_TREE<CIRCLE>( cn );
		p->m_pObject = &CAry[cn];	// 登録
		spOFTAry[cn].SetPtr(p);
	}


	///////////////////////////////////
	// 線形4分木マネージャ
	//  空間範囲をX=-60～720; Y=-1200～520に設定
	//  円が飛び出さない範囲を指定すれば良いので
	//  アバウトです
	CLiner4TreeManager<CIRCLE> LTree;
	if(!LTree.Init(g_PartitionLebel, -60.0f, -1200.0f, 720.0f, 520.0f))
	{
		MessageBox(NULL,_T("線形4分木空間の初期化に失敗しました。"),NULL,NULL);
		return 0;
	}

	// ウィンドウ表示
	ShowWindow( hWnd, SW_SHOW );

	// ループ内一時変数
	DWORD ColNum;				// 衝突判定回数
	CollisionList<CIRCLE>* ColVect;	// 衝突対象リスト
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

			// 円の位置を更新してツリーに再登録
			static bool change = true;
			for( cn = 0; cn < g_CircleNum; cn++ )
			{
				int i = cn;
				// ↓テストのために逸脱再登録の順番を変えています
				if ( change == true ) {
					i = g_CircleNum - cn - 1;
				}
				CIRCLE *pTmp = spOFTAry[i]->m_pObject;
				pTmp->pTex = pCircleTex;	// テクスチャ初期化
				GetNextCirclePos( *pTmp );	// 次の移動位置を仮決定
				spOFTAry[i]->Remove();		// 一度リストから外れる
				// 再登録
				LTree.Regist( pTmp->x-pTmp->r, pTmp->y-pTmp->r, pTmp->x+pTmp->r, pTmp->y+pTmp->r, spOFTAry[i].GetPtr() );
			}
			change = !change;

			// 衝突対応リストを取得
			ColNum = LTree.GetAllCollisionList( &ColVect );

			// 衝突判定
			DWORD c;
			ColNum/=2;	// 2で割るのはペアになっているので
			CIRCLE** pRoot = ColVect->getRootPtr();
			for(c=0; c<ColNum; c++){
				float r2 = (pRoot[c*2]->r+pRoot[c*2+1]->r)*(pRoot[c*2]->r+pRoot[c*2+1]->r);
				float x = (pRoot[c*2]->x-pRoot[c*2+1]->x);
				float y = (pRoot[c*2]->y-pRoot[c*2+1]->y);
				if(r2 >= x*x + y*y )
				{
					// ぶつかった物同士のテクスチャをオレンジに変更
					pRoot[c*2]->pTex = pColTex;
					pRoot[c*2+1]->pTex = pColTex;

					// 2円衝突処理をする
					CircleColProc( pRoot[c*2], pRoot[c*2+1] );
				}
			}

			g_pD3DDev->BeginScene();

			// 描画
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

			// ツリー情報表示
			_stprintf_s( pText, _T("Circle Number = %d"), g_CircleNum );
			drawText( pText, 0, 22 );
			_stprintf_s(pText, _T("Collision Check Number = %d"), ColNum/2);
			drawText( pText, 0, 44 );
			_stprintf_s(pText, _T("Optimaization = %2.5f ％"), 100.0*(float)(ColNum/2)/(g_CircleNum/2*(g_CircleNum+1)));
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