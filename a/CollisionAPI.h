// 衝突API群

// 修正情報
// 2007. 7. 20
// パーティクル衝突判定関数を更新

#pragma once

#pragma comment( lib, "d3dx9.lib" )
#include <d3dx9.h>

namespace IKD
{

///////////////////////////////////////////////////
// パーティクル衝突判定・時刻・位置算出関数
//   rA          : パーティクルAの半径
//   rB          : パーティクルBの半径
//   pre_pos_A   : パーティクルAの前の位置
//   pos_A       : パーティクルAの次の到達位置
//   pre_pos_B   : パーティクルBの前位置
//   pos_B       : パーティクルBの次の到達位置
//   pout_t      : 衝突時間を格納するFLOAT型へのポインタ
//   pout_colli_A : パーティクルAの衝突位置を格納するD3DXVECTOR型へのポインタ
//   pout_colli_B : パーティクルAの衝突位置を格納するD3DXVECTOR型へのポインタ

bool CalcParticleCollision(
   FLOAT rA, FLOAT rB, 
   D3DXVECTOR3 *pPre_pos_A, D3DXVECTOR3 *pPos_A,
   D3DXVECTOR3 *pPre_pos_B, D3DXVECTOR3 *pPos_B,
   FLOAT *pOut_t,
   D3DXVECTOR3 *pOut_colli_A,
   D3DXVECTOR3 *pOut_colli_B
)
{
   // 前位置及び到達位置におけるパーティクル間のベクトルを算出
   D3DXVECTOR3 C0 = *pPre_pos_B - *pPre_pos_A;
   D3DXVECTOR3 C1 = *pPos_B - *pPos_A;
   D3DXVECTOR3 D = C1 - C0;

   // 衝突判定用の2次関数係数の算出
   FLOAT P = D3DXVec3LengthSq( &D ); if(P==0) return false; // 同じ方向に移動
   FLOAT Q = D3DXVec3Dot( &C0, &D );
   FLOAT R = D3DXVec3LengthSq( &C0 );

   // パーティクル距離
   FLOAT r = rA + rB;

   // 衝突判定式
   FLOAT Judge = Q*Q - P*(R-r*r);
   if( Judge < 0 ){
      // 衝突していない
      return false;
   }

   // 衝突時間の算出
   FLOAT t_plus = (-Q + sqrt(Judge))/P;
   FLOAT t_minus = (-Q - sqrt(Judge))/P;

   // 衝突時間が0未満1より大きい場合、衝突しない
   if( (t_plus < 0 || t_plus > 1) && (t_minus < 0 || t_minus > 1)) return false;

   // 衝突時間の決定（t_minus側が常に最初の衝突）
   *pOut_t = t_minus;

   // 衝突位置の決定
   *pOut_colli_A = *pPre_pos_A + t_minus * (*pPos_A - *pPre_pos_A);
   *pOut_colli_B = *pPre_pos_B + t_minus * (*pPos_B - *pPre_pos_B);

   return true; // 衝突報告
}




///////////////////////////////////////////////////
// パーティクル衝突後速度位置算出関数
//   pColliPos_A : 衝突中のパーティクルAの中心位置
//   pVelo_A     : 衝突の瞬間のパーティクルAの速度
//   pColliPos_B : 衝突中のパーティクルBの中心位置
//   pVelo_B     : 衝突の瞬間のパーティクルBの速度
//   weight_A    : パーティクルAの質量
//   weight_B    : パーティクルBの質量
//   res_A       : パーティクルAの反発率
//   res_B       : パーティクルBの反発率
//   time        : 反射後の移動可能時間
//   pOut_pos_A  : パーティクルAの反射後位置
//   pOut_velo_A : パーティクルAの反射後速度ベクトル
//   pOut_pos_B  : パーティクルBの反射後位置
//   pOut_velo_B : パーティクルBの反射後速度ベクトル
bool CalcParticleColliAfterPos(
   D3DXVECTOR3 *pColliPos_A, D3DXVECTOR3 *pVelo_A,
   D3DXVECTOR3 *pColliPos_B, D3DXVECTOR3 *pVelo_B,
   FLOAT weight_A, FLOAT weight_B,
   FLOAT res_A, FLOAT res_B,
   FLOAT time,
   D3DXVECTOR3 *pOut_pos_A, D3DXVECTOR3 *pOut_velo_A,
   D3DXVECTOR3 *pOut_pos_B, D3DXVECTOR3 *pOut_velo_B
)
{
   FLOAT TotalWeight = weight_A + weight_B; // 質量の合計
   FLOAT RefRate = (1 + res_A*res_B); // 反発率
   D3DXVECTOR3 C = *pColliPos_B - *pColliPos_A; // 衝突軸ベクトル
   D3DXVec3Normalize(&C, &C);
   FLOAT Dot = D3DXVec3Dot( &(*pVelo_A-*pVelo_B), &C ); // 内積算出
   D3DXVECTOR3 ConstVec = RefRate*Dot/TotalWeight * C; // 定数ベクトル

   // 衝突後速度ベクトルの算出
   *pOut_velo_A = -weight_B * ConstVec + *pVelo_A;
   *pOut_velo_B = weight_A * ConstVec + *pVelo_B;

   // 衝突後位置の算出
   *pOut_pos_A = *pColliPos_A + time * (*pOut_velo_A);
   *pOut_pos_B = *pColliPos_B + time * (*pOut_velo_B);

   return true;
}


#define IKD_EPSIRON 0.00001f // 誤差

///////////////////////////////////////////////////
// 平面パーティクル衝突判定・時刻・位置算出関数
// r : パーティクルの半径
// pPre_pos : パーティクルの前の位置
// pPos : パーティクルの次の到達位置
// pNormal : 平面の法線
// pPlane_pos : 平面上の1点
// pOut_t : 衝突時間を格納するFLOAT型へのポインタ
// pOut_colli : パーティクルの衝突位置を格納するD3DXVECTOR型へのポインタ
// 戻り値 : 衝突(true), 非衝突(false)

bool CalcParticlePlaneCollision(
   FLOAT r,
   D3DXVECTOR3 *pPre_pos, D3DXVECTOR3 *pPos,
   D3DXVECTOR3 *pNormal, D3DXVECTOR3 *pPlane_pos,
   FLOAT *t,
   D3DXVECTOR3 *pOut_colli
) 
{
   D3DXVECTOR3 C0 = *pPre_pos - *pPlane_pos; // 平面上の一点から現在位置へのベクトル
   D3DXVECTOR3 D = *pPos - *pPre_pos; // 現在位置から予定位置までのベクトル
   D3DXVECTOR3 N; // 法線
   D3DXVec3Normalize(&N, pNormal); // 法線を標準化

   // 平面と中心点の距離を算出
   FLOAT Dot_C0 = D3DXVec3Dot( &C0, &N );
   FLOAT dist_plane_to_point = fabs( Dot_C0 );

   // 進行方向と法線の関係をチェック
   FLOAT Dot = D3DXVec3Dot( &D, &N );

   // 平面と平行に移動してめり込んでいるスペシャルケース
   if( (IKD_EPSIRON-fabs(Dot) > 0.0f) && (dist_plane_to_point < r) ){
      // 一生抜け出せないので最大時刻を返す
      *t = FLT_MAX;
      // 衝突位置は仕方ないので今の位置を指定
      *pOut_colli = *pPre_pos;
      return true;
   }

   // 交差時間の算出
   *t = ( r - Dot_C0 )/Dot;

   // 衝突位置の算出
   *pOut_colli = *pPre_pos + (*t) * D;

   // めり込んでいたら衝突として処理終了
   if ( dist_plane_to_point < r )
      return true;

   // 壁に対して移動が逆向きなら衝突していない
   if( Dot >= 0 )
      return false;

   // 時間が0～1の間にあれば衝突
   if( (0 <= *t) && (*t <= 1) )
      return true;

   return false;
}


///////////////////////////////////////////////////
// 平面と球の衝突後速度算出関数
// pColliPos : 衝突中のパーティクルの中心位置
// pVelo : 衝突の瞬間のパーティクルの速度
// res : パーティクルの壁に対する反発率
// time : 反射後の移動可能時間
// pNormal : 平面の法線
// pOut_pos : パーティクルの反射後位置
// pOut_velo : パーティクルの反射後速度ベクトル

bool CalcParticlePlaneAfterPos(
	D3DXVECTOR3 *pColliPos,
	D3DXVECTOR3 *pVelo,
	FLOAT res,
	FLOAT time,
	D3DXVECTOR3 *pNormal,
	D3DXVECTOR3 *pOut_pos,
	D3DXVECTOR3 *pOut_velo
)
{
	// 反射後速度を算出
	D3DXVECTOR3 N;
	D3DXVec3Normalize(&N,pNormal);
	*pOut_velo = *pVelo - (1+res)*D3DXVec3Dot(&N,pVelo)*N;

	// 移動位置を計算
	*pOut_pos = *pColliPos + *pOut_velo * time;

	return true;
}


}	// end namespace IKD