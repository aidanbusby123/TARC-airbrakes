#include<Arduino.h>


/*class Vec3 {
  private:
    // vector components
    float x;
    float y;
    float z;
    
  public:

    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
    //set vector components
    void setX(float x) { this->x = x;}
    void setY(float y) { this->y = y;}
    void setZ(float z) { this->z = z;}

    // get vector components

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    // get magnitude of vector

    float magnitude() const {
      return sqrt(x*x + y*y + z*z);
    }


    // normalize vector

    Vec3 normalize() const {
      float mag = magnitude();
      if (mag == 0){
        Serial.println("Error: cannot normalize a zero vector!");
      }
      return Vec3(x/mag, y/mag, z/mag);
    }
 

    // get dot product

    float dot(const Vec3& b) const {
      return this->x * b.x + this->y * b.y + this->z * b.z;
    }

    // get cross product
  
    Vec3 operator*(const Vec3& b) const {
      return Vec3(
        this->y * b.z - this->z * b.y,
        this->z * b.x - this->x * b.z,
        this->x * b.y - this->y * b.x
       );
    }

    // add vector
    
    Vec3 operator+(const Vec3& b) const {
      return Vec3(this->x + b.x, this->y + b.y, this->z + b.z);
    }

    // subtract vector
    
    Vec3 operator-(const Vec3& b) const {
      return Vec3(this->x - b.x, this->y - b.y, this->z - b.z);
    }

    // scale vector
    Vec3 operator*(const float k) const {
      return Vec3(this->x * k, this->y * k, this->z * k);
    }
  
};

class Quaternion {
private:
    float w, x, y, z; // Components of the quaternion

public:
    // Constructors
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {} // Default identity quaternion
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}


    // Set values

    void setW(float w) {this->w = w;}
    void setX(float x) {this->x = x;}
    void setY(float y) {this->y = y;}
    void setZ(float z) {this->z = z;}

    // Accessors
    float getW() const { return w; }
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    // Conjugate
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Magnitude
    float magnitude() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }

    // Normalization
    Quaternion normalize() const {
        float mag = magnitude();
        if (mag == 0.0f) {
            Serial.println("Cannot normalize a quaternion with zero magnitude.");
        }
        return Quaternion(w / mag, x / mag, y / mag, z / mag);
    }

    // Quaternion multiplication
    Quaternion operator*(const Quaternion &q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Scalar multiplication
    Quaternion operator*(const float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // Quaternion addition
    Quaternion operator+(const Quaternion &q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }


};
*/