#ifndef RECASTCONFIG_H
#define RECASTCONFIG_H

// Defines the number of bits allocated to rcSpan::smin and rcSpan::smax.
// If the mesh is appearing below the actual terrain, this parameter may need to be increased.
// rcSpan::smin�����rcSpan::smax�Ɋ��蓖�Ă���r�b�g�����`���܂��B
//���b�V�������ۂ̒n�`�̉��ɕ\�������ꍇ�A���̃p�����[�^�[�𑝂₷�K�v������ꍇ������܂��B
constexpr int RC_SPAN_HEIGHT_BITS = 13;

#endif // RECASTCONFIG_H