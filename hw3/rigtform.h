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
#endif
