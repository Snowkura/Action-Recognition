/*
 * �R�c����̍�����A�J�f�~�b�N�p�b�N�̃v���O�����̃��b�p�[
 * ������2�s�ŎԈ֎q�������o���D�X�S�C�I
 * Author: suzuryo Date: 2015.5.26
 * Modified:
 *  2015.5.27 - Modify Read/Write method
 *            - Create function: stop
 *            - Create function: toCJmode
 *                *** jwvehicle_serial_protocol.cpp, jwvehicle_serial_protocol.cpp ��CJ�R�}���h�ɑΉ�����悤�ɕύX
 *            - Modify destructor: �I������CJ�R�}���h���Ă�ŃA�J�f�~�b�N���[�h����W���C�X�e�B�b�N���[�h�ɕύX����悤�ɂ���
 *            - Modify move: 臒l��ݒ�
 */
#ifndef __JWVEHICLE_WRAPPER_H__
#define __JWVEHICLE_WRAPPER_H__

#include<Windows.h>
#include<process.h>

#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#include "jwvehicle_serial.hpp"
#include "jwvehicle_serial_protocol.hpp"

namespace jwvehicle{
	class jwvehicle_wrapper{
		device_t dev;
		OVERLAPPED ovWaitComm;
		OVERLAPPED ovRead;
		OVERLAPPED ovWrite;

		//�񓯊������p
		bool isLoop;
		//HANDLE thRWHandle;
		HANDLE thReadHandle;
		HANDLE thWriteHandle;
		HANDLE thChkReachedHandle;
		HANDLE thTimerHandle;
		HANDLE stopEvent;
		HANDLE writeEvent;
		HANDLE readEvent;
		HANDLE reachedEvent;
		HANDLE timerEvent;

		CRITICAL_SECTION wcs;
		char wlen;
		char wbuf[1024];
		CRITICAL_SECTION rcs;
		char rlen;
		char rbuf[1024];
		
		//�R�}���h���o�Ԋu
		enum{CMD_SPAN=50};

	public:
		//�R���X�g���N�^�F�C���X�^���X�����Ɠ����ɃV���A���ʐM���J�n���C�A�J�f�~�b�N���[�h�Ɉڍs����D
		//comNo: �J��COM�|�[�g timeout: �ڑ������̃^�C���A�E�g����[ms]�D���̎��Ԃ��߂���ƃv���O�������I��������D
		jwvehicle_wrapper(char *comNo, unsigned int timeout=3000);
		//�f�X�g���N�^�F�V���A���ʐM���I������D�W���C�X�e�B�b�N���샂�[�h�ɖ߂��D
		~jwvehicle_wrapper();

		//�Ԉ֎q�𓮂���
		//-100~100(%)�Ŋ����w��
		//vel: �O�i���x turn: ��]���x(��:����])
		void move(int vel, int turn);
		//�Ԉ֎q���~�߂�
		void stop();
		
		//�X���b�h�p�R�[���o�b�N�֐������[�U�͎g��Ȃ�
		friend static unsigned int __stdcall runThread_Read(void *t_this);
		friend static unsigned int __stdcall runThread_Write(void *t_this);
		friend static unsigned int __stdcall runThread_ChkReached(void *t_this);
		friend static unsigned int __stdcall runThread_Timer(void *t_this);

	private:
		//���M(���M�o�b�t�@�ɃR�s�[����)
		int write(char *buf);
		//��M(��M�o�b�t�@����R�s�[����)
		int read(char *buf);
		//������M(��M�o�b�t�@�ɏ������܂��܂Ńu���b�L���O)
		//timeout: �u���b�L���O�̃^�C���A�E�g���Ԃ̎w��[ms]
		int readOnBlock(char *buf,int timeout=INFINITE);

		//�A�J�f�~�b�N���[�h�Ɉڍs����D�A�J�f�~�b�N���[�h�ɂȂ�܂Ńu���b�N����D
		void toCAmode();
		//�W���C�X�e�B�b�N���[�h�Ɉڍs����D�W���C�X�e�B�b�N���[�h�ɂȂ�܂Ńu���b�N����D
		void toCJmode();

		void startRead();
		void startWrite();
		void startChkReached();
	};

}
//�ʖ�
typedef jwvehicle::jwvehicle_wrapper JWVehicle;

#endif //__JWVEHICLE_WRAPPER_H__
