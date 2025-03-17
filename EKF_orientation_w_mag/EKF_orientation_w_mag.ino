#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <quaternions.h>
#include <linalg_funcs.h>
#include <other_funcs.h>

typedef struct {
  float x[4];          // Current quaternion state [q0, q1, q2, q3]
  float xp[4];         // Predicted state (quaternion)
  float P_data[16] = {0};
  float Pp_data[16];
  float sigma_w_data[9] = {0};
  float R_data[36] = {0};
} ekf_t;

ekf_t ekf;

Adafruit_ICM20948 icm;

const float gravity = 10.015;

unsigned long t1 = 0, t2 = 0;
int counter = 0;
const int limit = 10;  

float east[3] = {0};
float north[3] = {0};

bool adjusted = false;
bool adjust_start = true;
int stationary_counter = 0;

float a[3] = {0};
float g[3] = {0};
float m[3] = {0};

// offset values
float ao[3] = {0};
float go[3] = {0};
float mo[3] = {0};

//global quaternions
struct quaternion q_r;
struct quaternion Q;

bool is_stationary(int counter, float (&arr)[3]) {
  if (abs(get_norm(arr) - gravity) < 0.15) {
    if (counter == 10) {
      return true;
    }
    else {
      counter += 1;
      return false;
    }
  }
  else {
    counter = 0;
    return false;
  }
}

//matrix functions
void matMultiply(const float *A, const float *B, float *C, int m, int n, int p) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < p; j++) {
      C[i * p + j] = 0.0f;
      for (int k = 0; k < n; k++) {
        C[i * p + j] += A[i * n + k] * B[k * p + j];
      }
    }
  }
}

void matTranspose(const float *A, float *At, int m, int n) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      At[j * m + i] = A[i * n + j];
    }
  }
}

void matAdd(const float *A, const float *B, float *C, int m, int n) {
  int total = m * n;
  for (int i = 0; i < total; i++) {
    C[i] = A[i] + B[i];
  }
}

void matSub(const float *A, const float *B, float *C, int m, int n) {
  int total = m * n;
  for (int i = 0; i < total; i++) {
    C[i] = A[i] - B[i];
  }
}

int matInverse3x3(const float *A, float *invA) {
  float a11 = A[0], a12 = A[1], a13 = A[2];
  float a21 = A[3], a22 = A[4], a23 = A[5];
  float a31 = A[6], a32 = A[7], a33 = A[8];
  
  float det = a11*(a22*a33 - a23*a32) - a12*(a21*a33 - a23*a31) + a13*(a21*a32 - a22*a31);
  if (fabs(det) < 1e-6) return -1;
  float invDet = 1.0f / det;
  
  invA[0] = (a22 * a33 - a23 * a32) * invDet;
  invA[1] = (a13 * a32 - a12 * a33) * invDet;
  invA[2] = (a12 * a23 - a13 * a22) * invDet;
  invA[3] = (a23 * a31 - a21 * a33) * invDet;
  invA[4] = (a11 * a33 - a13 * a31) * invDet;
  invA[5] = (a13 * a21 - a11 * a23) * invDet;
  invA[6] = (a21 * a32 - a22 * a31) * invDet;
  invA[7] = (a12 * a31 - a11 * a32) * invDet;
  invA[8] = (a11 * a22 - a12 * a21) * invDet;
  
  return 0;
}

int matInverse6x6(const float *A, float *Ainv)
{
    float M[36], I[36];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            M[i*6 + j] = A[i*6 + j];
            I[i*6 + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    for (int c = 0; c < 6; c++) {
        int pivot = c;
        float maxAbs = fabsf(M[c*6 + c]);
        for (int r = c + 1; r < 6; r++) {
            float val = fabsf(M[r*6 + c]);
            if (val > maxAbs) {
                pivot = r;
                maxAbs = val;
            }
        }
        if (maxAbs < 1e-6f) {
            return -1;
        }
        if (pivot != c) {
            for (int k = 0; k < 6; k++) {
                float tmpM = M[c*6 + k];
                M[c*6 + k] = M[pivot*6 + k];
                M[pivot*6 + k] = tmpM;

                float tmpI = I[c*6 + k];
                I[c*6 + k] = I[pivot*6 + k];
                I[pivot*6 + k] = tmpI;
            }
        }
        float pivotVal = M[c*6 + c];
        for (int k = 0; k < 6; k++) {
            M[c*6 + k] /= pivotVal;
            I[c*6 + k] /= pivotVal;
        }
      
        for (int r = c + 1; r < 6; r++) {
            float factor = M[r*6 + c];
            for (int k = 0; k < 6; k++) {
                M[r*6 + k] -= factor * M[c*6 + k];
                I[r*6 + k] -= factor * I[c*6 + k];
            }
        }
    }

    for (int c = 5; c >= 0; c--) {
        for (int r = c - 1; r >= 0; r--) {
            float factor = M[r*6 + c];
            for (int k = 0; k < 6; k++) {
                M[r*6 + k] -= factor * M[c*6 + k];
                I[r*6 + k] -= factor * I[c*6 + k];
            }
        }
    }

    for (int i = 0; i < 36; i++) {
        Ainv[i] = I[i];
    }

    return 0;

void quat_normalize(float q[4]) {
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm > 0.0f) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

void ekf_init(ekf_t* ekf, float sigma_wx, float sigma_wy, float sigma_wz, 
              float sigma_ax, float sigma_ay, float sigma_az, float sigma_mx, float sigma_my, float sigma_mz) {
  ekf->x[0] = 1.0f;
  ekf->x[1] = 0.0f;
  ekf->x[2] = 0.0f;
  ekf->x[3] = 0.0f;

  ekf->P_data[0]  = 1.0f;
  ekf->P_data[5]  = 1.0f;
  ekf->P_data[10] = 1.0f;
  ekf->P_data[15] = 1.0f;
  
  ekf->sigma_w_data[0] = sigma_wx * sigma_wx;
  ekf->sigma_w_data[4] = sigma_wy * sigma_wy;
  ekf->sigma_w_data[8] = sigma_wz * sigma_wz;

  ekf->R_data[0] = sigma_ax * sigma_ax;
  ekf->R_data[7] = sigma_ay * sigma_ay;
  ekf->R_data[14] = sigma_az * sigma_az;
  ekf->R_data[21] = sigma_mx * sigma_mx;
  ekf->R_data[28] = sigma_my * sigma_my;
  ekf->R_data[35] = sigma_mz * sigma_mz;
}

void ekf_predict(ekf_t *ekf, float w_x, float w_y, float w_z, float dt) {

  ekf->xp[0] = ekf->x[0] - 0.5f * (w_x * ekf->x[1] + w_y * ekf->x[2] + w_z * ekf->x[3]) * dt;
  ekf->xp[1] = ekf->x[1] + 0.5f * (w_x * ekf->x[0] - w_y * ekf->x[3] + w_z * ekf->x[2]) * dt;
  ekf->xp[2] = ekf->x[2] + 0.5f * (w_x * ekf->x[3] + w_y * ekf->x[0] - w_z * ekf->x[1]) * dt;
  ekf->xp[3] = ekf->x[3] + 0.5f * (-w_x * ekf->x[2] + w_y * ekf->x[1] + w_z * ekf->x[0]) * dt;
  quat_normalize(ekf->xp);

  float F[16] = {
      1.0f,            -0.5f * w_x * dt, -0.5f * w_y * dt, -0.5f * w_z * dt,
      0.5f * w_x * dt,  1.0f,             0.5f * w_z * dt, -0.5f * w_y * dt,
      0.5f * w_y * dt, -0.5f * w_z * dt,  1.0f,             0.5f * w_x * dt,
      0.5f * w_z * dt,  0.5f * w_y * dt, -0.5f * w_x * dt,  1.0f
  };
  float Ft[16];
  matTranspose(F, Ft, 4, 4);

  float W[12] = {
      -ekf->xp[1] * dt / 2, -ekf->xp[2] * dt / 2, -ekf->xp[3] * dt / 2,
       ekf->xp[0] * dt / 2, -ekf->xp[3] * dt / 2,  ekf->xp[2] * dt / 2,
       ekf->xp[3] * dt / 2,  ekf->xp[0] * dt / 2, -ekf->xp[1] * dt / 2,
      -ekf->xp[2] * dt / 2,  ekf->xp[1] * dt / 2,  ekf->xp[0] * dt / 2
  };

  float Wsigma[12];
  matMultiply(W, ekf->sigma_w_data, Wsigma, 4, 3, 3);

  float Wt[12];
  matTranspose(W, Wt, 4, 3);
  float Q[16];
  matMultiply(Wsigma, Wt, Q, 4, 3, 4);

  float FP[16];
  matMultiply(F, ekf->P_data, FP, 4, 4, 4);
  float FPFt[16];
  matMultiply(FP, Ft, FPFt, 4, 4, 4);
  float Pp[16];
  matAdd(FPFt, Q, Pp, 4, 4);
  for (int i = 0; i < 16; i++) {
    ekf->Pp_data[i] = Pp[i];
  }
}

void ekf_update(ekf_t *ekf, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z) {
  float a_norm = sqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  if (a_norm > 0.0f) {
    a_x /= a_norm;
    a_y /= a_norm;
    a_z /= a_norm;
  }

  float m_norm = sqrt(m_x*m_x + m_y*m_y + m_z*m_z);
  if (m_norm > 0.0f) {
    m_x /= m_norm;
    m_y /= m_norm;
    m_z /= m_norm;
  }
  float z[6] = { a_x, a_y, a_z, m_x, m_y, m_z};

  float r[3] = {0, cos(73.0f * M_PI/180.0f), -sin(73.0f * M_PI / 180.0f)};

  float h[6] = {
    2.0f * (ekf->xp[1] * ekf->xp[3] - ekf->xp[0] * ekf->xp[2]),
    2.0f * (ekf->xp[0] * ekf->xp[1] + ekf->xp[2] * ekf->xp[3]),
    2.0f * (0.5f - ekf->xp[1] * ekf->xp[1] - ekf->xp[2] * ekf->xp[2]),
    2.0f * (r[0] * (0.5f - ekf->xp[2] * ekf->xp[2] - ekf->xp[3] * ekf->xp[3]) + r[1] * (ekf->xp[0] * ekf->xp[3] + ekf->xp[1] * ekf->xp[2]) + r[2] * (ekf->xp[1] * ekf->xp[3] - ekf->xp[0] * ekf->xp[2])),
    2.0f * (r[0] * (ekf->xp[1] * ekf->xp[2] - ekf->xp[0] * ekf->xp[3]) + r[1] * (0.5f - ekf->xp[1] * ekf->xp[1] - ekf->xp[3] * ekf->xp[3]) + r[2] * (ekf->xp[0] * ekf->xp[1] + ekf->xp[2] * ekf->xp[3])),
    2.0f * (r[0] * (ekf->xp[0] * ekf->xp[2] + ekf->xp[1] * ekf->xp[3]) + r[1] * (ekf->xp[2] * ekf->xp[3] - ekf->xp[0] * ekf->xp[1]) + r[2] * (0.5f - ekf->xp[1] * ekf->xp[1] - ekf->xp[2] * ekf->xp[2]))
    };

  float v[6];
  for (int i = 0; i < 6; i++) {
    v[i] = z[i] - h[i];
  }

  float H[24] = {
    -2.0f * ekf->xp[2],  2.0f * ekf->xp[3], -2.0f * ekf->xp[0],  2.0f * ekf->xp[1],
     2.0f * ekf->xp[1],  2.0f * ekf->xp[0],  2.0f * ekf->xp[3],  2.0f * ekf->xp[2],
     2.0f * ekf->xp[0], -2.0f * ekf->xp[1], -2.0f * ekf->xp[2],  2.0f * ekf->xp[3],

     2.0f * (r[0] * ekf->xp[0] + r[1] * ekf->xp[3] - r[2] * ekf->xp[2]),
     2.0f * (r[0] * ekf->xp[1] + r[1] * ekf->xp[2] + r[2] * ekf->xp[3]),
     2.0f * (-r[0] * ekf->xp[2] + r[1] * ekf->xp[1] - r[2] * ekf->xp[0]),
     2.0f * (-r[0] * ekf->xp[3] + r[1] * ekf->xp[0] + r[2] * ekf->xp[1]),

     2.0f * (-r[0] * ekf->xp[3] + r[1] * ekf->xp[0] + r[2] * ekf->xp[1]),
     2.0f * (r[0] * ekf->xp[2] - r[1] * ekf->xp[1] + r[2] * ekf->xp[0]),
     2.0f * (r[0] * ekf->xp[1] + r[1] * ekf->xp[2] + r[2] * ekf->xp[3]),
     2.0f * (-r[0] * ekf->xp[0] - r[1] * ekf->xp[3] + r[2] * ekf->xp[2]),

     2.0f * (r[0] * ekf->xp[2] - r[1] * ekf->xp[1] + r[2] * ekf->xp[0]),
     2.0f * (r[0] * ekf->xp[3] - r[1] * ekf->xp[0] - r[2] * ekf->xp[1]),
     2.0f * (r[0] * ekf->xp[0] + r[1] * ekf->xp[3] - r[2] * ekf->xp[2]), 
     2.0f * (r[0] * ekf->xp[1] + r[1] * ekf->xp[2] + r[2] * ekf->xp[3])
  };

  float Ht[24];
  matTranspose(H, Ht, 6, 4);
  float PHt[24];
  matMultiply(ekf->Pp_data, Ht, PHt, 4, 4, 6);
  float S[36];
  matMultiply(H, PHt, S, 6, 4, 6);
  for (int i = 0; i < 36; i++) {
    S[i] += ekf->R_data[i];
  }
  float Si[36];
  if (matInverse6x6(S, Si) != 0) {
    return;
  }
  float K[24];
  matMultiply(PHt, Si, K, 4, 6, 6);

  float Kv[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      Kv[i] += K[i*6 + j] * v[j];
    }
    ekf->x[i] = ekf->xp[i] + Kv[i];
  }

  float KH[16];
  matMultiply(K, H, KH, 4, 6, 4);
  float I_KH[16];
  for (int i = 0; i < 16; i++) {
    int row = i / 4;
    int col = i % 4;
    float I_val = (row == col) ? 1.0f : 0.0f;
    I_KH[i] = I_val - KH[i];
  }
  float P_new[16];
  matMultiply(I_KH, ekf->Pp_data, P_new, 4, 4, 4);
  for (int i = 0; i < 16; i++) {
    ekf->P_data[i] = P_new[i];
  }
  
  quat_normalize(ekf->x);
}

void ekf_process(ekf_t *ekf, float w_x, float w_y, float w_z,
                 float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float dt) {
  Serial.print(ekf->x[0]); Serial.print(" ");
  Serial.print(ekf->x[1]); Serial.print(" ");
  Serial.print(ekf->x[2]); Serial.print(" ");
  Serial.println(ekf->x[3]);

  ekf_predict(ekf, w_x, w_y, w_z, dt);
  ekf_update(ekf, a_x, a_y, a_z, m_x, m_y, m_z);
}

void computeInitialOrientation(float ax, float ay, float az,
                               float mx, float my, float mz,
                               float q_out[4])
{
  float a_norm = sqrtf(ax*ax + ay*ay + az*az);
  if (a_norm < 1e-6f) return;
  ax /= a_norm; ay /= a_norm; az /= a_norm;

  float m_norm = sqrtf(mx*mx + my*my + mz*mz);
  if (m_norm < 1e-6f) return;
  mx /= m_norm; my /= m_norm; mz /= m_norm;

  float dx = -ax, dy = -ay, dz = -az;

  float ex = my*dz - mz*dy;
  float ey = mz*dx - mx*dz;
  float ez = mx*dy - my*dx;
  float e_norm = sqrtf(ex*ex + ey*ey + ez*ez);
  if (e_norm < 1e-6f) return;
  ex /= e_norm; ey /= e_norm; ez /= e_norm;

  float nx = dy*ez - dz*ey;
  float ny = dz*ex - dx*ez;
  float nz = dx*ey - dy*ex;

  float R[9];
  R[0] = ex; R[1] = nx; R[2] = dx;
  R[3] = ey; R[4] = ny; R[5] = dy;
  R[6] = ez; R[7] = nz; R[8] = dz;

  float trace = R[0] + R[4] + R[8];
  float qw, qx, qy, qz;
  if (trace > 0.0f) {
    float s = 0.5f / sqrtf(trace + 1.0f);
    qw = 0.25f / s;
    qx = (R[7] - R[5]) * s;
    qy = (R[2] - R[6]) * s;
    qz = (R[3] - R[1]) * s;
  } else {
    if (R[0] > R[4] && R[0] > R[8]) {
      float s = 2.0f * sqrtf(1.0f + R[0] - R[4] - R[8]);
      qw = (R[7] - R[5]) / s;
      qx = 0.25f * s;
      qy = (R[1] + R[3]) / s;
      qz = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
      float s = 2.0f * sqrtf(1.0f + R[4] - R[0] - R[8]);
      qw = (R[2] - R[6]) / s;
      qx = (R[1] + R[3]) / s;
      qy = 0.25f * s;
      qz = (R[5] + R[7]) / s;
    } else {
      float s = 2.0f * sqrtf(1.0f + R[8] - R[0] - R[4]);
      qw = (R[3] - R[1]) / s;
      qx = (R[2] + R[6]) / s;
      qy = (R[5] + R[7]) / s;
      qz = 0.25f * s;
    }
  }

  float normQ = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
  q_out[0] = qw / normQ;
  q_out[1] = qx / normQ;
  q_out[2] = qy / normQ;
  q_out[3] = qz / normQ;
}

void setup() {
  Wire.begin(33,32);
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case ICM20948_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20948_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20948_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
    case ICM20948_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
  }

  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);
  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);
  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
    case AK09916_MAG_DATARATE_SHUTDOWN:
      Serial.println("Shutdown");
      break;
    case AK09916_MAG_DATARATE_SINGLE:
      Serial.println("Single/One shot");
      break;
    case AK09916_MAG_DATARATE_10_HZ:
      Serial.println("10 Hz");
      break;
    case AK09916_MAG_DATARATE_20_HZ:
      Serial.println("20 Hz");
      break;
    case AK09916_MAG_DATARATE_50_HZ:
      Serial.println("50 Hz");
      break;
    case AK09916_MAG_DATARATE_100_HZ:
      Serial.println("100 Hz");
      break;
  }
  Serial.println();
    ekf_init(&ekf,
           /* gyro std dev */ 0.015f, 0.015f, 0.015f,
           /* accel std dev */ 0.01f, 0.01f, 0.01f,
           /* mag std dev */   0.05f,    0.05f,    0.05f);

  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float mx = mag.magnetic.x;
  float my = mag.magnetic.y;
  float mz = mag.magnetic.z;

  float q_init[4];
  computeInitialOrientation(ax, ay, az, mx, my, mz, q_init);

  ekf.x[0] = q_init[0];
  ekf.x[1] = q_init[1];
  ekf.x[2] = q_init[2];
  ekf.x[3] = q_init[3];
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

    a[0] = accel.acceleration.x;
    a[1] = accel.acceleration.y;
    a[2] = accel.acceleration.z;
    g[0] = gyro.gyro.x;
    g[1] = gyro.gyro.y;
    g[2] = gyro.gyro.z;
    m[0] = mag.magnetic.x;
    m[1] = mag.magnetic.y;
    m[2] = mag.magnetic.z;

    t1 = t2;
    t2 = micros();
    float t_f = (t2 - t1) / 1000000.0;

    cross(m, a, east);
    cross(a, east, north);
    unit(east);
    unit(north);

    ekf_process(&ekf, g[0], g[1], g[2], a[0], a[1], a[2], m[0], m[1], m[2], t_f);

    //print_data(north);

    Serial.print(ekf.x[0], 4); Serial.print(",");
    Serial.print(ekf.x[1], 4); Serial.print(",");
    Serial.print(ekf.x[2], 4); Serial.print(",");
    Serial.println(ekf.x[3], 4);

}
