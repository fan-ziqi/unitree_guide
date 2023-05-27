/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

KeyBoard::KeyBoard()
{
	// 初始化
	userCmd = UserCommand::NONE;
	userValue.setZero();

	// 保存当前终端属性到 _oldSettings。fileno() 函数返回指定文件流的文件描述符，stdin 是一个标准输入流，也就是用户从键盘输入的数据流
	tcgetattr(fileno(stdin), &_oldSettings);
	// 将 _newSettings 初始化为 _oldSettings
	_newSettings = _oldSettings;
	// 将 _newSettings 的 c_lflag 字段设置为非规范模式，即关闭缓冲区和回显功能
	_newSettings.c_lflag &= (~ICANON & ~ECHO);
	// 将终端属性设置为 _newSettings。TCSANOW 参数指示将属性更改应用于立即生效。
	tcsetattr(fileno(stdin), TCSANOW, &_newSettings);

	// 创建一个新线程，该线程运行 runKeyBoard 函数，将当前对象的指针作为参数传递。_tid 是线程 ID
	pthread_create(&_tid, NULL, runKeyBoard, (void *) this);
}

KeyBoard::~KeyBoard()
{
	// 将_id对应的线程标记为取消状态,使运行中的线程终止
	pthread_cancel(_tid);
	// 等待_id对应的线程终止
	pthread_join(_tid, NULL);

	// 将标准输入的属性设置为_oldSettings所保存的属性。重新启用标准输入的标准属性，包括缓冲输入和回显输入。
	tcsetattr(fileno(stdin), TCSANOW, &_oldSettings);
}

UserCommand KeyBoard::checkCmd()
{
	switch(_c)
	{
		case '1':
			return UserCommand::L2_B;
		case '2':
			return UserCommand::L2_A;
		case '3':
			return UserCommand::L2_X;
		case '4':
			return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
			case '5':
				return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
		case '0':
			return UserCommand::L1_X;
		case '9':
			return UserCommand::L1_A;
		case '8':
			return UserCommand::L1_Y;
		case '7':
			return UserCommand::BACKFLIP;
		case 't':
			return UserCommand::TOWR;
		case 'w':
			return UserCommand::WBC;
		case ' ':
			userValue.setZero();
			return UserCommand::NONE;
		default:
			return UserCommand::NONE;
	}
}

void KeyBoard::changeValue()
{
	switch(_c)
	{
		case 'w':
		case 'W':
			userValue.ly = min<float>(userValue.ly + sensitivityLeft, 1.0);
			break;
		case 's':
		case 'S':
			userValue.ly = max<float>(userValue.ly - sensitivityLeft, -1.0);
			break;
		case 'd':
		case 'D':
			userValue.lx = min<float>(userValue.lx + sensitivityLeft, 1.0);
			break;
		case 'a':
		case 'A':
			userValue.lx = max<float>(userValue.lx - sensitivityLeft, -1.0);
			break;

		case 'i':
		case 'I':
			userValue.ry = min<float>(userValue.ry + sensitivityRight, 1.0);
			break;
		case 'k':
		case 'K':
			userValue.ry = max<float>(userValue.ry - sensitivityRight, -1.0);
			break;
		case 'l':
		case 'L':
			userValue.rx = min<float>(userValue.rx + sensitivityRight, 1.0);
			break;
		case 'j':
		case 'J':
			userValue.rx = max<float>(userValue.rx - sensitivityRight, -1.0);
			break;
		default:
			break;
	}
}

void *KeyBoard::runKeyBoard(void *arg)
{
	((KeyBoard *) arg)->run(NULL);
	return NULL;
}

void *KeyBoard::run(void *arg)
{
	while(1)
	{
		// 将set清空
		FD_ZERO(&set);
		// 将stdin对应的文件描述符加入到set中
		FD_SET(fileno(stdin), &set);

		// 监听stdin是否有数据可读
		res = select(fileno(stdin) + 1, &set, NULL, NULL, NULL);

		// stdin有数据可读
		if(res > 0)
		{
			// 从stdin中读取一个字符
			ret = read(fileno(stdin), &_c, 1);
			// 检查读取的字符是否对应到一个用户指令
			userCmd = checkCmd();
			// 如果读取的字符不是一个用户指令，则改变用户操作的参数值
			if(userCmd == UserCommand::NONE)
			{
				changeValue();
			}
			// 将_c设置为空字符，以便下一次读取
			_c = '\0';
		}
		// 稍等一段时间，避免忙等
		usleep(1000);
	}
	return NULL;
}