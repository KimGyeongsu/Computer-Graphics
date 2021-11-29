#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    //TODO
      for (int i = 0; i < 3; ++i) {
          t_[i] = t[i];
      }
      for (int i = 0; i < 4; ++i) {
          r_[i] = r[i];
      }
  }

  explicit RigTForm(const Cvec3& t) {
    // TODO
      for (int i = 0; i < 3; ++i) {
          t_[i] = t[i];
      }
      r_ = Quat();
  }

  explicit RigTForm(const Quat& r) {
    // TODO
      t_ = Cvec3();
      for (int i = 0; i < 4; ++i) {
          r_[i] = r[i];
      }
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
      return r_ * a + Cvec4(t_, 0) * a[3];
    // TODO

  }

  RigTForm operator * (const RigTForm& a) const {
    // TODO
      RigTForm r = RigTForm();
      r.setRotation( r_ * a.getRotation());
      Cvec4 temp = r_ * Cvec4(a.getTranslation(), 0);
      Cvec3 temp2 = Cvec3();
      for (int i = 0; i < 3; i++) {
          temp2[i] = temp[i];
      }
      r.setTranslation(  t_ + temp2);
      return r;
  }
};

inline RigTForm inv(const RigTForm& tform) {
  // TODO
    RigTForm r = RigTForm();
    r.setRotation(inv(tform.getRotation()));
    Cvec4 temp = inv(tform.getRotation()) * Cvec4(tform.getTranslation(), 0);
    Cvec3 temp2 = Cvec3();
    for (int i = 0; i < 3; i++) {
        temp2[i] = - temp[i];
    }
    r.setTranslation( temp2);
    return r;
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  // TODO
    Matrix4 m = quatToMatrix(tform.getRotation());
    Cvec3 t = tform.getTranslation();
    for (int i = 0; i < 3; ++i) {
        m(i,3) = t[i];
    }
    return m;
}

inline RigTForm makeMixedFrame(const RigTForm& O, const RigTForm& E) {
    return transFact(O) * linFact(E);
}
inline RigTForm doMtoOwrtA(const RigTForm& A, const RigTForm& M, const RigTForm& O) {
    return A * M * inv(A) * O;
}

inline RigTForm interpolate(const RigTForm& a, const RigTForm& b, float i) {
    if (i == 0) return a;
    if (i == 1) return b;
    RigTForm r = RigTForm();

    r.setTranslation(a.getTranslation()* (1 - i) + b.getTranslation()*i);

    Quat q = b.getRotation() * inv(a.getRotation());

    if (q[0] < 0) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
      

    r.setRotation(power(q,i)* a.getRotation());

    return r;
}

inline RigTForm CRS_interpolate(const RigTForm& c0, const RigTForm& c1, const RigTForm& c2, const RigTForm& c3, float i) {
    if (abs(i - 0) < CS175_EPS) return c1;
    if (abs(i - 1) < CS175_EPS) return c2;

    RigTForm r = RigTForm();

    Cvec3 c0_t = c0.getTranslation();
    Cvec3 c1_t = c1.getTranslation();
    Cvec3 c2_t = c2.getTranslation();
    Cvec3 c3_t = c3.getTranslation();

    Cvec3 d_t = (c2_t - c0_t) / 6.0 + c1_t;
    Cvec3 e_t = (c1_t - c3_t) / 6.0 + c2_t;

    r.setTranslation(c1_t * pow((1 - i), 3) + d_t * (3*i*pow((1-i),2)) + e_t * (3 * (1-i) * pow(i, 2)) + c2_t * pow(i, 3));



    Quat c0_r = c0.getRotation();
    Quat c1_r = c1.getRotation();
    Quat c2_r = c2.getRotation();
    Quat c3_r = c3.getRotation();

    Quat d_r = power(cn(c2_r * inv(c0_r)), 1 / 6.0) * c1_r;
    Quat e_r = power(cn(c1_r * inv(c3_r)), 1 / 6.0) * c2_r;

    Quat p01_r = power(cn(d_r * inv(c1_r)), i) * c1_r;
    Quat p12_r = power(cn(e_r * inv(d_r)), i) * d_r;
    Quat p23_r = power(cn(c2_r * inv(e_r)), i) * e_r;

    Quat p012_r = power(cn(p12_r * inv(p01_r)), i) * p01_r;
    Quat p123_r = power(cn(p23_r * inv(p12_r)), i) * p12_r;

    r.setRotation(power(cn(p123_r * inv(p012_r)), i) * p012_r);
    return r;
}




#endif
