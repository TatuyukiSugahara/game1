// �Փ�API�Q

// �C�����
// 2007. 7. 20
// �p�[�e�B�N���Փ˔���֐����X�V

#pragma once

#pragma comment( lib, "d3dx9.lib" )
#include <d3dx9.h>

namespace IKD
{

///////////////////////////////////////////////////
// �p�[�e�B�N���Փ˔���E�����E�ʒu�Z�o�֐�
//   rA          : �p�[�e�B�N��A�̔��a
//   rB          : �p�[�e�B�N��B�̔��a
//   pre_pos_A   : �p�[�e�B�N��A�̑O�̈ʒu
//   pos_A       : �p�[�e�B�N��A�̎��̓��B�ʒu
//   pre_pos_B   : �p�[�e�B�N��B�̑O�ʒu
//   pos_B       : �p�[�e�B�N��B�̎��̓��B�ʒu
//   pout_t      : �Փˎ��Ԃ��i�[����FLOAT�^�ւ̃|�C���^
//   pout_colli_A : �p�[�e�B�N��A�̏Փˈʒu���i�[����D3DXVECTOR�^�ւ̃|�C���^
//   pout_colli_B : �p�[�e�B�N��A�̏Փˈʒu���i�[����D3DXVECTOR�^�ւ̃|�C���^

bool CalcParticleCollision(
   FLOAT rA, FLOAT rB, 
   D3DXVECTOR3 *pPre_pos_A, D3DXVECTOR3 *pPos_A,
   D3DXVECTOR3 *pPre_pos_B, D3DXVECTOR3 *pPos_B,
   FLOAT *pOut_t,
   D3DXVECTOR3 *pOut_colli_A,
   D3DXVECTOR3 *pOut_colli_B
)
{
   // �O�ʒu�y�ѓ��B�ʒu�ɂ�����p�[�e�B�N���Ԃ̃x�N�g�����Z�o
   D3DXVECTOR3 C0 = *pPre_pos_B - *pPre_pos_A;
   D3DXVECTOR3 C1 = *pPos_B - *pPos_A;
   D3DXVECTOR3 D = C1 - C0;

   // �Փ˔���p��2���֐��W���̎Z�o
   FLOAT P = D3DXVec3LengthSq( &D ); if(P==0) return false; // ���������Ɉړ�
   FLOAT Q = D3DXVec3Dot( &C0, &D );
   FLOAT R = D3DXVec3LengthSq( &C0 );

   // �p�[�e�B�N������
   FLOAT r = rA + rB;

   // �Փ˔��莮
   FLOAT Judge = Q*Q - P*(R-r*r);
   if( Judge < 0 ){
      // �Փ˂��Ă��Ȃ�
      return false;
   }

   // �Փˎ��Ԃ̎Z�o
   FLOAT t_plus = (-Q + sqrt(Judge))/P;
   FLOAT t_minus = (-Q - sqrt(Judge))/P;

   // �Փˎ��Ԃ�0����1���傫���ꍇ�A�Փ˂��Ȃ�
   if( (t_plus < 0 || t_plus > 1) && (t_minus < 0 || t_minus > 1)) return false;

   // �Փˎ��Ԃ̌���it_minus������ɍŏ��̏Փˁj
   *pOut_t = t_minus;

   // �Փˈʒu�̌���
   *pOut_colli_A = *pPre_pos_A + t_minus * (*pPos_A - *pPre_pos_A);
   *pOut_colli_B = *pPre_pos_B + t_minus * (*pPos_B - *pPre_pos_B);

   return true; // �Փ˕�
}




///////////////////////////////////////////////////
// �p�[�e�B�N���Փˌ㑬�x�ʒu�Z�o�֐�
//   pColliPos_A : �Փ˒��̃p�[�e�B�N��A�̒��S�ʒu
//   pVelo_A     : �Փ˂̏u�Ԃ̃p�[�e�B�N��A�̑��x
//   pColliPos_B : �Փ˒��̃p�[�e�B�N��B�̒��S�ʒu
//   pVelo_B     : �Փ˂̏u�Ԃ̃p�[�e�B�N��B�̑��x
//   weight_A    : �p�[�e�B�N��A�̎���
//   weight_B    : �p�[�e�B�N��B�̎���
//   res_A       : �p�[�e�B�N��A�̔�����
//   res_B       : �p�[�e�B�N��B�̔�����
//   time        : ���ˌ�̈ړ��\����
//   pOut_pos_A  : �p�[�e�B�N��A�̔��ˌ�ʒu
//   pOut_velo_A : �p�[�e�B�N��A�̔��ˌ㑬�x�x�N�g��
//   pOut_pos_B  : �p�[�e�B�N��B�̔��ˌ�ʒu
//   pOut_velo_B : �p�[�e�B�N��B�̔��ˌ㑬�x�x�N�g��
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
   FLOAT TotalWeight = weight_A + weight_B; // ���ʂ̍��v
   FLOAT RefRate = (1 + res_A*res_B); // ������
   D3DXVECTOR3 C = *pColliPos_B - *pColliPos_A; // �Փˎ��x�N�g��
   D3DXVec3Normalize(&C, &C);
   FLOAT Dot = D3DXVec3Dot( &(*pVelo_A-*pVelo_B), &C ); // ���ώZ�o
   D3DXVECTOR3 ConstVec = RefRate*Dot/TotalWeight * C; // �萔�x�N�g��

   // �Փˌ㑬�x�x�N�g���̎Z�o
   *pOut_velo_A = -weight_B * ConstVec + *pVelo_A;
   *pOut_velo_B = weight_A * ConstVec + *pVelo_B;

   // �Փˌ�ʒu�̎Z�o
   *pOut_pos_A = *pColliPos_A + time * (*pOut_velo_A);
   *pOut_pos_B = *pColliPos_B + time * (*pOut_velo_B);

   return true;
}


#define IKD_EPSIRON 0.00001f // �덷

///////////////////////////////////////////////////
// ���ʃp�[�e�B�N���Փ˔���E�����E�ʒu�Z�o�֐�
// r : �p�[�e�B�N���̔��a
// pPre_pos : �p�[�e�B�N���̑O�̈ʒu
// pPos : �p�[�e�B�N���̎��̓��B�ʒu
// pNormal : ���ʂ̖@��
// pPlane_pos : ���ʏ��1�_
// pOut_t : �Փˎ��Ԃ��i�[����FLOAT�^�ւ̃|�C���^
// pOut_colli : �p�[�e�B�N���̏Փˈʒu���i�[����D3DXVECTOR�^�ւ̃|�C���^
// �߂�l : �Փ�(true), ��Փ�(false)

bool CalcParticlePlaneCollision(
   FLOAT r,
   D3DXVECTOR3 *pPre_pos, D3DXVECTOR3 *pPos,
   D3DXVECTOR3 *pNormal, D3DXVECTOR3 *pPlane_pos,
   FLOAT *t,
   D3DXVECTOR3 *pOut_colli
) 
{
   D3DXVECTOR3 C0 = *pPre_pos - *pPlane_pos; // ���ʏ�̈�_���猻�݈ʒu�ւ̃x�N�g��
   D3DXVECTOR3 D = *pPos - *pPre_pos; // ���݈ʒu����\��ʒu�܂ł̃x�N�g��
   D3DXVECTOR3 N; // �@��
   D3DXVec3Normalize(&N, pNormal); // �@����W����

   // ���ʂƒ��S�_�̋������Z�o
   FLOAT Dot_C0 = D3DXVec3Dot( &C0, &N );
   FLOAT dist_plane_to_point = fabs( Dot_C0 );

   // �i�s�����Ɩ@���̊֌W���`�F�b�N
   FLOAT Dot = D3DXVec3Dot( &D, &N );

   // ���ʂƕ��s�Ɉړ����Ă߂荞��ł���X�y�V�����P�[�X
   if( (IKD_EPSIRON-fabs(Dot) > 0.0f) && (dist_plane_to_point < r) ){
      // �ꐶ�����o���Ȃ��̂ōő厞����Ԃ�
      *t = FLT_MAX;
      // �Փˈʒu�͎d���Ȃ��̂ō��̈ʒu���w��
      *pOut_colli = *pPre_pos;
      return true;
   }

   // �������Ԃ̎Z�o
   *t = ( r - Dot_C0 )/Dot;

   // �Փˈʒu�̎Z�o
   *pOut_colli = *pPre_pos + (*t) * D;

   // �߂荞��ł�����Փ˂Ƃ��ď����I��
   if ( dist_plane_to_point < r )
      return true;

   // �ǂɑ΂��Ĉړ����t�����Ȃ�Փ˂��Ă��Ȃ�
   if( Dot >= 0 )
      return false;

   // ���Ԃ�0�`1�̊Ԃɂ���ΏՓ�
   if( (0 <= *t) && (*t <= 1) )
      return true;

   return false;
}


///////////////////////////////////////////////////
// ���ʂƋ��̏Փˌ㑬�x�Z�o�֐�
// pColliPos : �Փ˒��̃p�[�e�B�N���̒��S�ʒu
// pVelo : �Փ˂̏u�Ԃ̃p�[�e�B�N���̑��x
// res : �p�[�e�B�N���̕ǂɑ΂��锽����
// time : ���ˌ�̈ړ��\����
// pNormal : ���ʂ̖@��
// pOut_pos : �p�[�e�B�N���̔��ˌ�ʒu
// pOut_velo : �p�[�e�B�N���̔��ˌ㑬�x�x�N�g��

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
	// ���ˌ㑬�x���Z�o
	D3DXVECTOR3 N;
	D3DXVec3Normalize(&N,pNormal);
	*pOut_velo = *pVelo - (1+res)*D3DXVec3Dot(&N,pVelo)*N;

	// �ړ��ʒu���v�Z
	*pOut_pos = *pColliPos + *pOut_velo * time;

	return true;
}


}	// end namespace IKD