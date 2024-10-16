#include "keyboard.h"

void keyboard::InitializePublishers()
{
    key_publisher = nh.advertise<std_msgs::UInt16>("key_command", 10);
}

void keyboard::scanKeyboard()
{
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    key_command = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);
}
void keyboard::switchkeyboard()
{
    scanKeyboard();
    switch (key_command)
    {
    case KEYCODE_W:
        key_c.data = 'W';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_A:
        key_c.data = 'A';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_S:
        key_c.data = 'S';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_D:
        key_c.data = 'D';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_I:
        key_c.data = 'I';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_J:
        key_c.data = 'J';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_K:
        key_c.data = 'K';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_L:
        key_c.data = 'L';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_Q:
        key_c.data = 'Q';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_E:
        key_c.data = 'E';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_F:
        key_c.data = 'F';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_G:
        key_c.data = 'G';
        key_publisher.publish(key_c);
        break;

    case KEYCODE_H:
        key_c.data = 'H';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_X:
        key_c.data = 'X';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_Y:
        key_c.data = 'Y';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_Z:
        key_c.data = 'Z';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_C:
        key_c.data = 'C';
        key_publisher.publish(key_c);
        ros::shutdown();
        break;
    case KEYCODE_V:
        key_c.data = 'V';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_B:
        key_c.data = 'B';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_N:
        key_c.data = 'N';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_M:
        key_c.data = 'M';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_O:
        key_c.data = 'O';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_P:
        key_c.data = 'P';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_R:
        key_c.data = 'R';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_T:
        key_c.data = 'T';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_U:
        key_c.data = 'U';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_0:
        key_c.data = '0';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_1:
        key_c.data = '1';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_2:
        key_c.data = '2';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_3:
        key_c.data = '3';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_4:
        key_c.data = '4';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_5:
        key_c.data = '5';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_6:
        key_c.data = '6';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_7:
        key_c.data = '7';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_8:
        key_c.data = '8';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_9:
        key_c.data = '9';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_DENGHAO:
        key_c.data = '=';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_DOUHAO:
        key_c.data = ',';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_JUHAO:
        key_c.data = '.';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_JIANHAO:
        key_c.data = '-';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_FENHAO:
        key_c.data = ';';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_XIEGANG:
        key_c.data = '/';
        key_publisher.publish(key_c);
        break;
    case KEYCODE_SPACE:
        key_c.data = '(space)';
        key_publisher.publish(key_c);
        break;
    }
}