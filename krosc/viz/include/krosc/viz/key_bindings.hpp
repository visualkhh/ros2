#pragma once

#include <QKeyEvent>
#include <QString>

namespace krosc::viz {

// 한글 두벌식 레이아웃에서도 W/A/S/D/Q/Z/F/C가 같은 물리키로 동작하게 한다.
// QKeyEvent::key()가 Qt::Key_unknown이거나 자모(U+314x)로 들어와도 text()를 QWERTY로 되돌린다.
inline int qwerty_key(const QKeyEvent* event) {
  const int key = event->key();
  switch (key) {
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
    case Qt::Key_Q:
    case Qt::Key_Z:
    case Qt::Key_F:
    case Qt::Key_C:
    case Qt::Key_R:
    case Qt::Key_Up:
    case Qt::Key_Down:
    case Qt::Key_Left:
    case Qt::Key_Right:
    case Qt::Key_Escape:
    case Qt::Key_Delete:
      return key;
    default:
      break;
  }
  const QString text = event->text();
  if (text.size() != 1) {
    return key;
  }
  const ushort code = text.at(0).unicode();
  switch (code) {
    case 0x3142:  // ㅂ
      return Qt::Key_Q;
    case 0x3148:  // ㅈ
      return Qt::Key_W;
    case 0x3137:  // ㄷ
      return Qt::Key_E;
    case 0x3131:  // ㄱ
      return Qt::Key_R;
    case 0x3145:  // ㅅ
      return Qt::Key_T;
    case 0x315B:  // ㅛ
      return Qt::Key_Y;
    case 0x3155:  // ㅕ
      return Qt::Key_U;
    case 0x3151:  // ㅑ
      return Qt::Key_I;
    case 0x3150:  // ㅐ
      return Qt::Key_O;
    case 0x3154:  // ㅔ
      return Qt::Key_P;
    case 0x3141:  // ㅁ
      return Qt::Key_A;
    case 0x3134:  // ㄴ
      return Qt::Key_S;
    case 0x3147:  // ㅇ
      return Qt::Key_D;
    case 0x3139:  // ㄹ
      return Qt::Key_F;
    case 0x314E:  // ㅎ
      return Qt::Key_G;
    case 0x3157:  // ㅗ
      return Qt::Key_H;
    case 0x3153:  // ㅓ
      return Qt::Key_J;
    case 0x314F:  // ㅏ
      return Qt::Key_K;
    case 0x3163:  // ㅣ
      return Qt::Key_L;
    case 0x314B:  // ㅋ
      return Qt::Key_Z;
    case 0x314C:  // ㅌ
      return Qt::Key_X;
    case 0x314A:  // ㅊ
      return Qt::Key_C;
    case 0x314D:  // ㅍ
      return Qt::Key_V;
    case 0x3160:  // ㅠ
      return Qt::Key_B;
    case 0x315C:  // ㅜ
      return Qt::Key_N;
    case 0x3161:  // ㅡ
      return Qt::Key_M;
    default:
      break;
  }
  // 한글 IME가 자모를 완성형 음절로 커밋한 경우(예: "자") 초성으로 되돌린다.
  if (code >= 0xAC00 && code <= 0xD7A3) {
    const int initial = (code - 0xAC00) / (21 * 28);
    switch (initial) {
      case 0:   // ㄱ
      case 1:   // ㄲ
        return Qt::Key_R;
      case 2:   // ㄴ
        return Qt::Key_S;
      case 3:   // ㄷ
      case 4:   // ㄸ
        return Qt::Key_E;
      case 5:   // ㄹ
        return Qt::Key_F;
      case 6:   // ㅁ
        return Qt::Key_A;
      case 7:   // ㅂ
      case 8:   // ㅃ
        return Qt::Key_Q;
      case 9:   // ㅅ
      case 10:  // ㅆ
        return Qt::Key_T;
      case 11:  // ㅇ
        return Qt::Key_D;
      case 12:  // ㅈ
      case 13:  // ㅉ
        return Qt::Key_W;
      case 14:  // ㅊ
        return Qt::Key_C;
      case 15:  // ㅋ
        return Qt::Key_Z;
      case 16:  // ㅌ
        return Qt::Key_X;
      case 17:  // ㅍ
        return Qt::Key_V;
      case 18:  // ㅎ
        return Qt::Key_G;
      default:
        break;
    }
  }
  if (code >= 'a' && code <= 'z') {
    return Qt::Key_A + (code - 'a');
  }
  if (code >= 'A' && code <= 'Z') {
    return Qt::Key_A + (code - 'A');
  }
  return key;
}

inline bool is_move_key(int key) {
  return key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D ||
         key == Qt::Key_Up || key == Qt::Key_Down || key == Qt::Key_Left || key == Qt::Key_Right;
}

inline bool is_teleop_shortcut(int key) {
  return is_move_key(key) || key == Qt::Key_Q || key == Qt::Key_Z || key == Qt::Key_F ||
         key == Qt::Key_C || key == Qt::Key_R;
}

// 키 릴리즈는 text()가 비어 있을 수 있으므로, 물리키(스캔코드)별로 누른 키를 기억한다.
inline int physical_key(const QKeyEvent* event) {
  return event->nativeScanCode() != 0 ? static_cast<int>(event->nativeScanCode()) : event->key();
}

}  // namespace krosc::viz
