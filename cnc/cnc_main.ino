#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <MsTimer2.h>

#define PIN_SPI_MOSI  11
#define PIN_SPI_MISO  12
#define PIN_SPI_SCK   13
#define PIN_SPI_SS    10
#define PIN_BUSY      9
#define PIN_BUSY2     8
#define PIN_BUSY3     7

#define BASE_PT       32000
#define Z_POS         1 * BASE_PT
#define BASE_STEP     3L

#define DIA_NORMAL    1
#define DIA_REVERSE   0

#define MAX_TAG       50
#define MAX_TRANS     3

typedef struct {
  char tag[3];
  double val[4];
} TAG;

long cx, cy, cz;
long step_mode_x, step_mode_y, step_mode_z;
long total_step_x, total_step_y;
long sign_px, sign_py, sign_pz;

long base_step_x, base_step_y, base_step_z;
double base_rate_x, base_rate_y, base_rate_z;

void check_act(TAG t);
void goto_pos(long pos_x, long pos_y, long pos_z, int wait, bool is_end_x, bool is_end_y);
void set_param(char *act, long param_x, long param_y, long param_z);

void setup() {
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);
  pinMode(PIN_BUSY, INPUT_PULLUP);
  pinMode(PIN_BUSY2, INPUT_PULLUP);
  pinMode(PIN_BUSY3, INPUT_PULLUP);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  Serial.begin(38400);
  digitalWrite(PIN_SPI_SS, HIGH);

  L6470_resetdevice();
  L6470_resetdevice2();
  L6470_resetdevice3();

  L6470_setparam_acc(0xfff);
  L6470_setparam_dec(0xfff);
  L6470_setparam_maxspeed(0x20);
  L6470_setparam_minspeed(0x00);
  L6470_setparam_fsspd(0x027);
  L6470_setparam_kvalhold(0xFF);
  L6470_setparam_kvalrun(0xFF);
  L6470_setparam_kvalacc(0xFF);
  L6470_setparam_kvaldec(0xFF);
  L6470_setparam_stepmood(0x07);

  L6470_setparam_acc2(0xfff);
  L6470_setparam_dec2(0xfff);
  L6470_setparam_maxspeed2(0x20);
  L6470_setparam_minspeed2(0x00);
  L6470_setparam_fsspd2(0x027);
  L6470_setparam_kvalhold2(0xFF);
  L6470_setparam_kvalrun2(0xFF);
  L6470_setparam_kvalacc2(0xFF);
  L6470_setparam_kvaldec2(0xFF);
  L6470_setparam_stepmood2(0x07);

  L6470_setparam_acc3(0xfff);
  L6470_setparam_dec3(0xfff);
  L6470_setparam_maxspeed3(0x20);
  L6470_setparam_minspeed3(0x00);
  L6470_setparam_fsspd3(0x027);
  L6470_setparam_kvalhold3(0xFF);
  L6470_setparam_kvalrun3(0xFF);
  L6470_setparam_kvalacc3(0xFF);
  L6470_setparam_kvaldec3(0xFF);
  L6470_setparam_stepmood3(0x07);

  cx = 0;
  cy = 0;
  cz = 0;

  sign_px = DIA_NORMAL;
  sign_py = DIA_REVERSE;
  sign_pz = DIA_REVERSE;

  set_step_mode(7L, 7L, 7L);

  Serial.print("READY\n");
}

void loop() {
  TAG tag_list[MAX_TAG];
  static char act;
  char buf[128], *b, c, *b_pt, *t_pt, *t;
  static int read_flag = 0;
  int i, tag_cnt, wait_cnt, j;

  tag_cnt = 0;
  wait_cnt = 0;

  if (Serial.available() > 0) {
    memset(buf, '\0', sizeof(buf));
    i = Serial.readBytesUntil('\n', buf, sizeof(buf));
    buf[i] = '\0';

    read_flag = 0;

    b = strtok_r(buf, "|", &b_pt);
    while (b != NULL) {
      t = strtok_r(b, ",", &t_pt);

      memset(tag_list[tag_cnt].tag, '\0', 4);
      for (j = 0; j < 4; j++) {
        tag_list[tag_cnt].val[j] = 0;
      }

      j = 0;
      while (t != NULL && j < 5) {
        if (j == 0) {
          strcpy(tag_list[tag_cnt].tag, t);
        }
        else {
          tag_list[tag_cnt].val[j - 1] = atof(t);
        }

        t = strtok_r(NULL, ",", &t_pt);
        j++;
      }

      b = strtok_r(NULL, "|", &b_pt);
      tag_cnt++;
    }
  }

  for (i = 0; i < tag_cnt; i++) {
    check_act(tag_list[i]);

    Serial.print("OK/X=");
    Serial.print(cx);
    Serial.print("/Y=");
    Serial.print(cy);
    Serial.print("/Z=");
    Serial.print(cz);
    Serial.print("\n");
  }
}

void check_act(TAG t) {
  char act[3];
  long i, j, n;
  long param_x, param_y, param_z;
  long pos_x, pos_y, pos_z, pos_sx, pos_sy, pos_sz, pos_ix, pos_iy, pos_iz, top_z, pos_nx, pos_ny;
  long def_x, def_y, sign_x, sign_y;
  double grad;
  bool is_end_x, is_end_y;
  long pre_x, pre_y, dup_cnt_x, dup_cnt_y;

  strncpy(act, t.tag, 3);

  if (act[0] == '@') {
    param_x = (long)(t.val[0]);
    param_y = (long)(t.val[1]);
    param_z = (long)(t.val[2]);

    set_param(act, param_x, param_y, param_z);
  }
  else {
    pos_x = t.val[0];
    pos_y = t.val[1];
    pos_z = t.val[2];
    top_z = t.val[3];

    if (act[0] == 'S') {
      L6470_setparam_abspos(0);
      L6470_setparam_abspos2(0);
      L6470_setparam_abspos3(0);
      L6470_setparam_mark(0);
      L6470_setparam_mark2(0);
      L6470_setparam_mark3(0);

      cx = 0;
      cy = 0;
      cz = 0;
    }
    else if (act[0] == 'E') {
      if (cx != 0 || cy != 0) {
        goto_pos(cx, cy, top_z, 10, false, false);
        goto_pos(0, 0, top_z, 10, false, false);
      }
      goto_pos(0, 0, 0, 10, false, false);

      L6470_hardhiz();
      L6470_hardhiz2();
      L6470_hardhiz3();

      cx = 0;
      cy = 0;
      cz = 0;
    }
    else if (act[0] == 'G') {
      /*pos_pz = cz;
      goto_pos(cx, cy, 0, 10, false, false);
      goto_pos(pos_x,pos_y, 0, 10, false, false);
      goto_pos(cx, cy, pos_pz, 10, false, false);*/
      if (cx != pos_x || cy != pos_y) {
        goto_pos(cx, cy, top_z, 10, false, false);
        goto_pos(pos_x, pos_y, top_z, 10, false, false);
      }
      goto_pos(cx, cy, pos_z, 10, false, false);
    }
    else if (act[0] == 'M') {
      if (pos_z > cz) {
        /*
        if (cx != pos_x || cy != pos_y) {
          goto_pos(cx, cy, top_z, 10, false, false);
          goto_pos(pos_x,pos_y, top_z, 10, false, false);
        }
        */
        goto_pos(cx, cy, pos_z, 10, false, false);
      }

      if (pos_x != cx || pos_y != cy) {
        def_x = pos_x - cx;
        def_y = pos_y - cy;
        sign_x = (def_x > 0) ? 1 : -1;
        sign_y = (def_y > 0) ? 1 : -1;

        total_step_x = sign_x * def_x; // * BASE_STEP / step_mode_x;
        total_step_y = sign_y * def_y; // * BASE_STEP / step_mode_y;

        if (pos_x == cx) {
          goto_pos(cx, pos_y, cz, 0, false, false);
        }
        else if (pos_y == cy) {
          goto_pos(pos_x, cy, cz, 0, false, false);
        }
        else {
          grad = (labs(def_x) >= labs(def_y)) ? (double)def_y / (double)def_x : (double)def_x / (double)def_y;

          pos_sx = cx;
          pos_sy = cy;

          dup_cnt_x = 0;
          dup_cnt_y = 0;
          pre_x = cx;
          pre_y = cy;

          for (i = 1; cx != pos_x || cy != pos_y; i++) {
            if (labs(def_x) >= labs(def_y)) {
              pos_ix = pos_sx + (long)(sign_x * i);
              pos_iy = pos_sy + (long)(sign_x * grad * i);

              if (pos_iy == cy) {
                j = i + 1;
                while (true) {
                  pos_ny = pos_sy + (long)(sign_x * grad * j);

                  if (pos_ny == cy && fabs(pos_ny - pos_y) >= fabs(sign_x * grad)) {
                    j++;
                  }
                  else {
                    i = j - 1;
                    pos_ix = pos_sx + (long)(sign_x * i);
                    break;
                  }
                }
              }
            }
            else {
              pos_ix = pos_sx + (long)(sign_y * grad * i);
              pos_iy = pos_sy + (long)(sign_y * i);

              if (pos_ix == cx) {
                j = i + 1;
                while (true) {
                  pos_nx = pos_sx + (long)(sign_y * grad * j);

                  if (pos_nx == cx && fabs(pos_nx - pos_x) >= fabs(sign_y * grad)) {
                    j++;
                  }
                  else {
                    i = j - 1;
                    pos_iy = pos_sy + (long)(sign_y * i);
                    break;
                  }
                }
              }
            }

            is_end_x = false;
            if ((pos_ix - cx > 0 && pos_ix > pos_x) || (pos_ix - cx < 0 && pos_ix < pos_x)) {
              pos_ix = pos_x;
            }
            if (pos_x == pos_ix) {
              is_end_x = true;
            }

            is_end_y = false;
            if ((pos_iy - cy > 0 && pos_iy > pos_y) || (pos_iy - cy < 0 && pos_iy < pos_y)) {
              pos_iy = pos_y;
            }
            if (pos_y == pos_iy) {
              is_end_y = true;
            }

            goto_pos(pos_ix, pos_iy, cz, 0, is_end_x, is_end_y);
          }

          /*
          for (i = 1; cx != pos_x || cy != pos_y; i++) {
            if (labs(def_x) >= labs(def_y)) {
              pos_ix = pos_px + (long)(sign_x * i);
              pos_iy = pos_py + (long)(sign_x * i * grad);
            }
            else {
              pos_ix = pos_px + (long)(sign_y * i * grad);
              pos_iy = pos_py + (long)(sign_y * i);
            }

            is_end_x = false;
            if ((pos_ix - cx > 0 && pos_ix > pos_x) || (pos_ix - cx < 0 && pos_ix < pos_x)) {
              pos_ix = pos_x;
            }
            if (pos_x == pos_ix) {
              is_end_x = true;
            }

            is_end_y = false;
            if ((pos_iy - cy > 0 && pos_iy > pos_y) || (pos_iy - cy < 0 && pos_iy < pos_y)) {
              pos_iy = pos_y;
            }
            if (pos_y == pos_iy) {
              is_end_y = true;
            }

            if (pos_ix == pre_x && !is_end_x) {
              dup_cnt_x++;
            }
            else {
              if (dup_cnt_x >= 2) {
                goto_pos(pre_x, pre_y, cz, 0, is_end_x, is_end_y);
              }
              dup_cnt_x = 0;
            }

            if (pos_iy == pre_y && !is_end_y) {
              dup_cnt_y++;
            }
            else {
              if (dup_cnt_y >= 2) {
                goto_pos(pre_x, pre_y, cz, 0, is_end_x, is_end_y);
              }
              dup_cnt_y = 0;
            }

            if (dup_cnt_x < 2 && dup_cnt_y < 2) {
              goto_pos(pos_ix, pos_iy, cz, 0, is_end_x, is_end_y);
            }

            pre_x = pos_ix;
            pre_y = pos_iy;
          }*/
        }
      }

      if (pos_z < cz) {
        /*
        if (cx != pos_x || cy != pos_y) {
          goto_pos(cx, cy, top_z, 10, false, false);
          goto_pos(pos_x,pos_y, top_z, 10, false, false);
        }
        */
        goto_pos(cx, cy, pos_z, 10, false, false);
      }
    }
    else if (act[0] == 'Z') {
      goto_pos(cx, cy, cz + base_step_z * 5, 1000, false, false);;
      L6470_gountil3(1, 0, base_step_z / 2L);
      L6470_setparam_abspos(0);
      L6470_setparam_abspos2(0);
      L6470_setparam_abspos3(0);
      L6470_setparam_mark(0);
      L6470_setparam_mark2(0);
      L6470_setparam_mark3(0);
      cx = 0;
      cy = 0;
      cz = 0;
      L6470_busydelay3(100);
      goto_pos(0, 0, base_step_z * 5, 1000, false, false);
      goto_pos(0, 0, -base_step_z * 0.5, 0, false, false);
      L6470_setparam_abspos(0);
      L6470_setparam_abspos2(0);
      L6470_setparam_abspos3(0);
      L6470_setparam_mark(0);
      L6470_setparam_mark2(0);
      L6470_setparam_mark3(0);
      cx = 0;
      cy = 0;
      cz = 0;
    }
    else if (act[0] == 'm') {
      goto_pos(pos_x, pos_y, cz, 0, false, false);
    }
  }
}

void goto_pos(long pos_x, long pos_y, long pos_z, int wait, bool is_end_x, bool is_end_y) {
  int sign_x, sign_y, sign_z;
  long x, y, z;

  sign_x = (pos_x - cx > 0) ?  DIA_NORMAL : DIA_REVERSE;
  sign_y = (pos_y - cy > 0) ?  DIA_NORMAL : DIA_REVERSE;
  sign_z = (pos_z - cz > 0) ?  DIA_NORMAL : DIA_REVERSE;

  if (pos_x != cx) {
    if (is_end_x) {
      x = total_step_x;
      total_step_x = 0;
    }
    else {
      x = labs(pos_x - cx); // * BASE_STEP / step_mode_x;
      total_step_x -= x;
    }
    if (sign_x != sign_px) {
      x += (long)(0.5 * base_rate_x);
    }
    L6470_move(sign_x, x);
    sign_px = sign_x;
  }
  if (pos_y != cy) {
    if (is_end_y) {
      y = total_step_y;
      total_step_y = 0;
    }
    else {
      y = labs(pos_y - cy); // * BASE_STEP / step_mode_y;
      total_step_y -= y;
    }
    if (sign_y != sign_py) {
      y += (long)(0.1 * base_rate_y);
    }
    L6470_move2(sign_y, y);
    sign_py = sign_y;
  }
  if (pos_z != cz) {
    z = labs(pos_z - cz); // * BASE_STEP / step_mode_z;
    L6470_move3(sign_z, z);
    sign_pz = sign_z;
  }
  L6470_busydelay(0);
  L6470_busydelay2(0);
  L6470_busydelay3(wait);

  cx = pos_x;
  cy = pos_y;
  cz = pos_z;

  /*
  Serial.print("MV/X=");
  Serial.print(cx);
  Serial.print("/Y=");
  Serial.print(cy);
  Serial.print("/Z=");
  Serial.print(cz);
  Serial.print("\n");
  */
}

void set_param(char *act, long param_x, long param_y, long param_z) {
  if (strncmp(act, "@00", 3) == 0) {
    L6470_resetdevice();
    L6470_resetdevice2();
    L6470_resetdevice3();
  }
  if (strncmp(act, "@01", 3) == 0) {
    L6470_setparam_abspos(param_x);
    L6470_setparam_abspos2(param_y);
    L6470_setparam_abspos3(param_z);
  }
  else if (strncmp(act, "@02", 3) == 0) {
    L6470_setparam_elpos(param_x);
    L6470_setparam_elpos2(param_y);
    L6470_setparam_elpos3(param_z);
  }
  else if (strncmp(act, "@03", 3) == 0) {
    L6470_setparam_mark(param_x);
    L6470_setparam_mark2(param_y);
    L6470_setparam_mark3(param_z);
  }
  /*
  else if (strncmp(act, "@04", 3) == 0) {
  }
  */
  else if (strncmp(act, "@05", 3) == 0) {
    L6470_setparam_acc(param_x);
    L6470_setparam_acc2(param_y);
    L6470_setparam_acc3(param_z);
  }
  else if (strncmp(act, "@06", 3) == 0) {
    L6470_setparam_dec(param_x);
    L6470_setparam_dec2(param_y);
    L6470_setparam_dec3(param_z);
  }
  else if (strncmp(act, "@07", 3) == 0) {
    L6470_setparam_maxspeed(param_x);
    L6470_setparam_maxspeed2(param_y);
    L6470_setparam_maxspeed3(param_z / 2);
  }
  else if (strncmp(act, "@08", 3) == 0) {
    L6470_setparam_minspeed(param_x);
    L6470_setparam_minspeed2(param_y);
    L6470_setparam_minspeed3(param_z);
  }
  else if (strncmp(act, "@09", 3) == 0) {
    L6470_setparam_kvalhold(param_x);
    L6470_setparam_kvalhold2(param_y);
    L6470_setparam_kvalhold3(param_z);
  }
  else if (strncmp(act, "@0A", 3) == 0) {
    L6470_setparam_kvalrun(param_x);
    L6470_setparam_kvalrun2(param_y);
    L6470_setparam_kvalrun3(param_z);
  }
  else if (strncmp(act, "@0B", 3) == 0) {
    L6470_setparam_kvalacc(param_x);
    L6470_setparam_kvalacc2(param_y);
    L6470_setparam_kvalacc3(param_z);
  }
  else if (strncmp(act, "@0C", 3) == 0) {
    L6470_setparam_kvaldec(param_x);
    L6470_setparam_kvaldec2(param_y);
    L6470_setparam_kvaldec3(param_z);
  }
  else if (strncmp(act, "@0D", 3) == 0) {
    L6470_setparam_intspd(param_x);
    L6470_setparam_intspd2(param_y);
    L6470_setparam_intspd3(param_z);
  }
  else if (strncmp(act, "@0E", 3) == 0) {
    L6470_setparam_stslp(param_x);
    L6470_setparam_stslp2(param_y);
    L6470_setparam_stslp3(param_z);
  }
  else if (strncmp(act, "@0F", 3) == 0) {
    L6470_setparam_fnslpacc(param_x);
    L6470_setparam_fnslpacc2(param_y);
    L6470_setparam_fnslpacc3(param_z);
  }
  else if (strncmp(act, "@10", 3) == 0) {
    L6470_setparam_fnslpdec(param_x);
    L6470_setparam_fnslpdec2(param_y);
    L6470_setparam_fnslpdec3(param_z);
  }
  else if (strncmp(act, "@11", 3) == 0) {
    L6470_setparam_ktherm(param_x);
    L6470_setparam_ktherm2(param_y);
    L6470_setparam_ktherm3(param_z);
  }
  /*
  else if (strncmp(act, "@12", 3) == 0) {
  }
  */
  else if (strncmp(act, "@13", 3) == 0) {
    L6470_setparam_ocdth(param_x);
    L6470_setparam_ocdth2(param_y);
    L6470_setparam_ocdth3(param_z);
  }
  else if (strncmp(act, "@14", 3) == 0) {
    L6470_setparam_stallth(param_x);
    L6470_setparam_stallth2(param_y);
    L6470_setparam_stallth3(param_z);
  }
  else if (strncmp(act, "@15", 3) == 0) {
    L6470_setparam_fsspd(param_x);
    L6470_setparam_fsspd2(param_y);
    L6470_setparam_fsspd3(param_z);
  }
  else if (strncmp(act, "@16", 3) == 0) {
    L6470_setparam_stepmood(param_x);
    L6470_setparam_stepmood2(param_y);
    L6470_setparam_stepmood3(param_z);

    set_step_mode(param_x, param_y, param_z);
  }
  else if (strncmp(act, "@17", 3) == 0) {
    L6470_setparam_alareen(param_x);
    L6470_setparam_alareen2(param_y);
    L6470_setparam_alareen3(param_z);
  }
}

void set_step_mode(long mode_x, long mode_y, long mode_z) {
  step_mode_x = (long)pow(2, 8L - mode_x);
  step_mode_y = (long)pow(2, 8L - mode_y);
  step_mode_z = (long)pow(2, 8L - mode_z);

  base_step_x = 200L * (long)pow(2, mode_x);
  base_step_y = 200L * (long)pow(2, mode_y);
  base_step_z = 200L * (long)pow(2, mode_z);
  base_rate_x = (double)base_step_x / 1.496;
  base_rate_y = (double)base_step_y / 1.496;
  base_rate_z = (double)base_step_y / 1.496;
}
