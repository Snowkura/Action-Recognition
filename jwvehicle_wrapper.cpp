/*
 * �R�c����̍�����A�J�f�~�b�N�p�b�N�̃v���O�����̃��b�p�[
 * Author: suzuryo Date: 2015.5.26
 */

#include"jwvehicle_wrapper.h"

namespace jwvehicle{
	jwvehicle_wrapper::jwvehicle_wrapper(char *comNo, unsigned int timeout){
		//���낢�돀��
		InitializeCriticalSection(&wcs);
		InitializeCriticalSection(&rcs);

		ovWaitComm.hEvent=CreateEvent(NULL,FALSE,FALSE,NULL); ovWaitComm.Offset=0; ovWaitComm.OffsetHigh=0;
		ovRead.hEvent=CreateEvent(NULL,FALSE,FALSE,NULL); ovRead.Offset=0; ovRead.OffsetHigh=0;
		ovWrite.hEvent=CreateEvent(NULL,FALSE,FALSE,NULL); ovWrite.Offset=0; ovWrite.OffsetHigh=0;
		if(jwvehicle::open(&dev,comNo)!=0){
			fprintf(stderr,"Error on open %s\n",comNo);
			exit(-1);
		}
		if(SetCommMask(dev,EV_RXFLAG)==0){fprintf(stderr,"Err code %d\n",GetLastError());} //�ݒ肵���I�[�����܂œǂ񂾂�C�x���g����������
		Sleep(100);

		stopEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
		writeEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
		readEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
		reachedEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
		timerEvent=CreateEvent(NULL,TRUE,FALSE,NULL);

		//�X���b�h�J�n
		thTimerHandle=(HANDLE)_beginthreadex(NULL,0,runThread_Timer,this,0,NULL);
		thChkReachedHandle=(HANDLE)_beginthreadex(NULL,0,runThread_ChkReached,this,0,NULL);
		thReadHandle=(HANDLE)_beginthreadex(NULL,0,runThread_Read,this,0,NULL);
		thWriteHandle=(HANDLE)_beginthreadex(NULL,0,runThread_Write,this,0,NULL);
		Sleep(100);

		//�A�J�f�~�b�N���[�h�Ɉڍs
		toCAmode();
	}
	jwvehicle_wrapper::~jwvehicle_wrapper(){
		//�Ƃ肠�����~�߂�
		stop();
		Sleep(100);

		//�W���C�X�e�B�b�N���[�h�Ɉڍs
		toCJmode();

		//�X���b�h��~
		SetEvent(stopEvent);
		SetCommMask(dev,EV_RXCHAR); //WaitCommEvent�������E�o
		HANDLE events[4]={thReadHandle,thWriteHandle,thChkReachedHandle,thTimerHandle};
		WaitForMultipleObjects(4,events,TRUE,INFINITE);
		PurgeComm(dev,PURGE_RXCLEAR); //�ʐM�w�̃o�b�t�@���N���A

		//��n��
		jwvehicle::close(&dev);
		Sleep(100);

		CloseHandle(stopEvent);
		CloseHandle(writeEvent);
		CloseHandle(readEvent);
		CloseHandle(reachedEvent);
		CloseHandle(timerEvent);

		CloseHandle(thReadHandle);
		CloseHandle(thWriteHandle);
		CloseHandle(thChkReachedHandle);
		CloseHandle(thTimerHandle);

		CloseHandle(ovRead.hEvent);
		CloseHandle(ovWrite.hEvent);
		CloseHandle(ovWaitComm.hEvent);

		DeleteCriticalSection(&wcs);
		DeleteCriticalSection(&rcs);
	}

	//�Ԉ֎q�𓮂���
	//-100~100(%)�Ŋ����w��
	//vel: �O�i���x turn: ��]���x(+:����])
	void jwvehicle_wrapper::move(int vel, int turn){
		char buf[1024];
		if(vel<-100) vel=-100;
		else if(vel>100) vel=100;
		if(turn<-100) turn=-100;
		else if(turn>100) turn=100;
		cmd_createCS(buf, vel, turn);
		write(buf);
	}
	void jwvehicle_wrapper::stop(){
		move(0,0);
	}
		
	int jwvehicle_wrapper::write(char *buf){
		int len=strlen(buf)+1;
		EnterCriticalSection(&wcs);
			wlen=len;
			memcpy(wbuf,buf,wlen);
		LeaveCriticalSection(&wcs);
		SetEvent(writeEvent);
		return len;
	}
	int jwvehicle_wrapper::read(char *buf){
		int len=0;
		EnterCriticalSection(&rcs);
			len=rlen;
			memcpy(buf,rbuf,len);
			rlen=0;
		LeaveCriticalSection(&rcs);
		ResetEvent(readEvent);
		return len;
	}
	int jwvehicle_wrapper::readOnBlock(char *buf,int timeout){
		if(WaitForSingleObject(readEvent,timeout)==WAIT_TIMEOUT) return -1;
		ResetEvent(readEvent);
		return read(buf);
	}

	unsigned int __stdcall runThread_Read(void *t_this){
		jwvehicle_wrapper *_this=(jwvehicle_wrapper*)t_this;
		_this->startRead();
		_endthreadex(0);
		return 0;
	}
	unsigned int __stdcall runThread_Write(void *t_this){
		jwvehicle_wrapper *_this=(jwvehicle_wrapper*)t_this;
		_this->startWrite();
		_endthreadex(0);
		return 0;
	}
	unsigned int __stdcall runThread_ChkReached(void *t_this){
		jwvehicle_wrapper *_this=(jwvehicle_wrapper*)t_this;
		_this->startChkReached();
		_endthreadex(0);
		return 0;
	}

	unsigned int __stdcall runThread_Timer(void *t_this){
		jwvehicle_wrapper *_this=(jwvehicle_wrapper*)t_this;
		while(true){
			DWORD ret=WaitForSingleObject(_this->stopEvent,jwvehicle_wrapper::CMD_SPAN);
			if(ret!=WAIT_TIMEOUT) break;
			SetEvent(_this->timerEvent);
		}
		_endthreadex(0);
		return 0;
	}

	void jwvehicle_wrapper::startRead(){
		char buf[1024];
		HANDLE events[2]={stopEvent,reachedEvent};
		while(true){
			DWORD ret=WaitForMultipleObjects(2,events,FALSE,INFINITE);
			if(ret==STATUS_WAIT_0+0){ //stop
				break;
			}else if(ret==STATUS_WAIT_0+1){ //reached
				ResetEvent(reachedEvent);
				//��M
				int len=jwvehicle::read(dev,buf,1024,&ovRead);
				if(len==-1){
					DWORD err=0;
					if((err=GetLastError())==ERROR_IO_PENDING){ //�f�[�^������
						DWORD tsize=0;
						GetOverlappedResult(dev,&ovRead,&tsize,TRUE); //��������
						len=tsize;
						if(FAILED(err=GetLastError())){
							fprintf(stderr,"Error on jwvehicle::read 0x%x\n",err); continue;
						}
					}else{
						fprintf(stderr,"Error on jwvehicle::read 0x%x\n",err); continue;
					}
				}
				ResetEvent(ovRead.hEvent);

				ResetEvent(timerEvent); //�ǂ񂾂�^�C�}�[���Z�b�g
				if(len==-1){ fprintf(stderr,"Error on jwvehicle::read 0x%x\n",GetLastError()); continue;}
				if(len>0){
					//�o�b�t�@�ɃR�s�[
					EnterCriticalSection(&rcs);
						rlen=len;
						memcpy(rbuf,buf,rlen);
					LeaveCriticalSection(&rcs);
				}
				//for debug
				//for(int i=0;i<len;++i) putchar(buf[i]);
				//�ǂݍ��񂾂��Ƃ�ʒm
				SetEvent(readEvent);
			}
		}
	}
	void jwvehicle_wrapper::startWrite(){
		HANDLE events[2]={stopEvent,writeEvent};
		char buf[1024];
		int len=0;
		while(true){
			DWORD ret=WaitForMultipleObjects(2,events,FALSE,INFINITE);
			if(ret==STATUS_WAIT_0+0){ //stop
				break;
			}else if(STATUS_WAIT_0+1){ //write
				//�ǂݍ��񂾂��Ƃ�50ms�ȏ�҂�
				WaitForSingleObject(timerEvent,CMD_SPAN);
				//�o�b�t�@����R�s�[
				EnterCriticalSection(&wcs);
					len=wlen;
					if(len>0) memcpy(buf,wbuf,len);
					wlen=0;
				LeaveCriticalSection(&wcs);
				//���M
				if(len>0) if(jwvehicle::write(dev,buf,len,&ovWrite)==-1){
						DWORD err=GetLastError();
						if(err==ERROR_IO_PENDING){ //�f�[�^������
							DWORD tsize=0;
							GetOverlappedResult(dev,&ovWrite,&tsize,TRUE); //��������
							if(FAILED(err=GetLastError())) fprintf(stderr,"Error on jwvehicle::write 0x%x\n",err);
						}else{
							fprintf(stderr,"Error on jwvehicle::write 0x%x\n",err);
						}
					}
				ResetEvent(ovWrite.hEvent);
			}

		}
	}
	void jwvehicle_wrapper::startChkReached(){
		while(true){
			if(WaitForSingleObject(stopEvent,0)!=WAIT_TIMEOUT) break;
			DWORD emask=0;
			BOOL ret;
			BOOL ret2;
			DWORD err;
			if(!(ret=WaitCommEvent(dev,&emask,&ovWaitComm))) if((err=GetLastError())==ERROR_IO_PENDING){
				DWORD tSize;
				ret2=GetOverlappedResult(dev,&ovWaitComm,&tSize,TRUE);
				if(!ret2) printf("0x%x\n",GetLastError());
			}
			ResetEvent(ovWaitComm.hEvent);
			if(((!ret && err==ERROR_IO_PENDING)  || ret) && (emask&EV_RXFLAG)){
				SetEvent(reachedEvent);
			}
			if(err!=0 && err!=ERROR_IO_PENDING){
				printf("WaitCommEvent: Error code: 0x%x\n",err);
			}
		}
	}
	void jwvehicle_wrapper::toCAmode(){
		char wbuf[1024],rbuf[1024];
		cmd_createCA(wbuf);
		int wlen=strlen(wbuf);
		bool loop_end=false;
		while(!loop_end){
			Sleep(CMD_SPAN);
			//�ǂݏ���
			write(wbuf);
			rlen=readOnBlock(rbuf);
			if(rlen<=0) continue;

			int len=rlen; //�̂���̒���
			char cmd[1024];
			int hpos=cmd_findheader(rbuf); //�w�b�_�̈ʒu������
			if(hpos==-1) continue; //�w�b�_���Ȃ�������ǂݏ������s
			len-=hpos;
			memmove(rbuf,rbuf+hpos,len); //�w�b�_�̑O���폜
			while(true){
				int len_cmd=cmd_pop(cmd,rbuf,&len); //�o�b�t�@����R�}���h�E��
				if(len_cmd==-1) break; //�R�}���h���Ȃ�������ǂݏ������s
				else if(cmd_check(cmd,len_cmd)==CMD_RA)// goto loop_end; //�����������߂�
				{
					loop_end=true;
					break;
				}
			}
		}
		//loop_end:
	}

	void jwvehicle_wrapper::toCJmode(){
		char wbuf[1024],rbuf[1024];
		cmd_createCJ(wbuf);
		int wlen=strlen(wbuf);
		bool loop_end=false;
		while(!loop_end){
			Sleep(CMD_SPAN);
			//�ǂݏ���
			write(wbuf);
			rlen=readOnBlock(rbuf);
			if(rlen<=0) continue;

			int len=rlen; //�̂���̒���
			char cmd[1024];
			int hpos=cmd_findheader(rbuf); //�w�b�_�̈ʒu������
			if(hpos==-1) continue; //�w�b�_���Ȃ�������ǂݏ������s
			len-=hpos;
			memmove(rbuf,rbuf+hpos,len); //�w�b�_�̑O���폜
			while(true){
				int len_cmd=cmd_pop(cmd,rbuf,&len); //�o�b�t�@����R�}���h�E��
				if(len_cmd==-1) break; //�R�}���h���Ȃ�������ǂݏ������s
				else if(cmd_check(cmd,len_cmd)==CMD_RJ)// goto loop_end; //�����������߂�
				{
					loop_end=true;
					break;
				}
			}
		}
		//loop_end:
	}
}
