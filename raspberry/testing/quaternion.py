import numbers
import numpy as np


class Quaternion(numbers.Number):
    def __init__(self, w, x=None, y=None, z=None):
        if isinstance(w, np.ndarray):
            if x is None and y is None and z is None:
                self._w = 0.
                self._x = w.flatten()[0]
                self._y = w.flatten()[1]
                self._z = w.flatten()[2]
            elif (isinstance(x, np.ndarray) and isinstance(y, np.ndarray)
                    and isinstance(z, np.ndarray)):
                self._w = float(w)
                self._x = float(x)
                self._y = float(y)
                self._z = float(z)
        elif (isinstance(w, Quaternion) and
              (x is None and y is None and z is None)):
            self = w.copy()
        elif (isinstance(w, numbers.Complex)):
            if (x is None and y is None and z is None):
                self._w = w.real
                self._x = w.imag
                self._y = 0.
                self._z = 0.
            elif isinstance(x, np.ndarray) and w.imag == 0:
                self._w = w.real
                self._x = x.flatten()[0]
                self._y = x.flatten()[1]
                self._z = x.flatten()[2]
            elif isinstance(x, Quaternion) and x.w == 0:
                self._w = w
                self._x = x.x
                self._y = x.y
                self._z = x.z
            else:
                try:
                    self._w = float(w)
                    self._x = float(x)
                    self._y = float(y)
                    self._z = float(z)
                except Exception:
                    raise TypeError('Wrong input type!')

    def __add__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(self._w + other._w, self._x + other._x,
                              self._y + other._y, self._z + other._z)
        elif isinstance(other, numbers.Complex):
            return Quaternion(self._w + other.real, self._x + other.imag,
                              self._y, self._z)
        else:
            return NotImplemented

    def __sub__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(self._w - other._w, self._x - other._x,
                              self._y - other._y, self._z - other._z)
        elif isinstance(other, numbers.Complex):
            return Quaternion(self._w - other.real, self._x - other.imag,
                              self._y, self._z)
        else:
            return NotImplemented

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(self._w * other._w - self._x * other._x -
                              self._y * other._y - self._z * other._z,
                              self._w * other._x + self._x * other._w +
                              self._y * other._z - self._z * other._y,
                              self._w * other._y - self._x * other._z +
                              self._y * other._w + self._z * other._x,
                              self._w * other._z + self._x * other._y -
                              self._y * other._x + self._z * other._w)
        elif isinstance(other, numbers.Complex):
            return Quaternion(self._w * other.real - self._x * other.imag,
                              self._w * other.imag + self._x * other.real,
                              self._y * other.real + self._z * other.imag,
                              -self._y * other.imag + self._z * other.real)
        else:
            return NotImplemented

    def __truediv__(self, other):
        if isinstance(other, Quaternion):
            return self * (other.conjugate()/other.norm())
        elif isinstance(other, numbers.Complex):
            return self * (other.conjugate()/(other*other.conjugate()))
        else:
            return NotImplemented

    def __pow__(self, other):
        if (isinstance(other, Quaternion) or
                isinstance(other, numbers.Complex)):
            return (other * self.ln()).exp()
        else:
            return NotImplemented

    def __radd__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(self._w + other._w, self._x + other._x,
                              self._y + other._y, self._z + other._z)
        elif isinstance(other, numbers.Complex):
            return Quaternion(self._w + other.real, self._x + other.imag,
                              self._y, self._z)
        else:
            return NotImplemented

    def __rsub__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(other._w - self._w, other._x - self._x,
                              other._y - self._y, other._z - self._z)
        elif isinstance(other, numbers.Complex):
            return Quaternion(other.real - self._w, other.imag - self._x,
                              -self._y, -self._z)
        else:
            return NotImplemented

    def __rmul__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(other._w * self._w - other._x * self._x -
                              other._y * self._y - other._z * self._z,
                              other._w * self._x + other._x * self._w +
                              other._y * self._z - other._z * self._y,
                              other._w * self._y - other._x * self._z +
                              other._y * self._w + other._z * self._x,
                              other._w * self._z + other._x * self._y -
                              other._y * self._x + other._z * self._w)
        elif isinstance(other, numbers.Complex):
            return Quaternion(other.real * self._w - other.imag * self._x,
                              other.real * self._x + other.imag * self._w,
                              other.real * self._y - other.imag * self._z,
                              other.real * self._z + other.imag * self._y)
        else:
            return NotImplemented

    def __rtruediv__(self, other):
        if (isinstance(other, Quaternion) or
                isinstance(other, numbers.Complex)):
            return other * self.inverse()
        else:
            return NotImplemented

    def __rpow__(self, other):
        if isinstance(other, Quaternion):
            return (self * other.ln()).exp()
        elif isinstance(other, numbers.Complex):
            return (self * np.log(other)).exp()
        else:
            return NotImplemented

    def __iadd__(self, other):
        if isinstance(other, Quaternion):
            self._w += other._w
            self._x += other._x
            self._y += other._y
            self._z += other._z
            return self
        elif isinstance(other, numbers.Complex):
            self._w += other.real
            self._x += other.imag
            return self
        else:
            return NotImplemented

    def __isub__(self, other):
        if isinstance(other, Quaternion):
            self._w -= other._w
            self._x -= other._x
            self._y -= other._y
            self._z -= other._z
            return self
        elif isinstance(other, numbers.Complex):
            self._w -= other.real
            self._x -= other.imag
            return self
        else:
            return NotImplemented

    def __imul__(self, other):
        if isinstance(other, Quaternion):
            self = Quaternion(self._w * other._w + self._x * other._x +
                              self._y * other._y + self._z * other._z,
                              self._w * other._x + self._x * other._w +
                              self._y * other._z - self._z * other._y,
                              self._w * other._y - self._x * other._z +
                              self._y * other._w + self._z * other._x,
                              self._w * other._z + self._x * other._y -
                              self._y * other._x + self._z * other._w)
            return self
        elif isinstance(other, numbers.Complex):
            self = Quaternion(self._w * other.real + self._x * other.imag,
                              self._w * other.imag + self._x * other.real,
                              self._y * other.real + self._z * other.imag,
                              self._y * other.imag + self._z * other.real)
            return self
        else:
            return NotImplemented

    def __itruediv__(self, other):
        if isinstance(other, Quaternion):
            self *= (other.conjugate()/other.norm())
            return self
        elif isinstance(other, numbers.Complex):
            self *= (other.conjugate()/(other*other.conjugate()))
            return self
        else:
            return NotImplemented

    def __ipow__(self, other):
        if (isinstance(other, Quaternion) or
                isinstance(other, numbers.Complex)):
            self = (other * self.ln()).exp()
            return self
        else:
            return NotImplemented

    def __neg__(self):
        return -1 * self

    def __pos__(self):
        return self

    def __abs__(self):
        return self.norm()

    def inverse(self):
        return (self.conjugate()/np.square(self.norm()))

    def exp(self):
        v = self.v
        norm = v.norm()
        if norm != 0:
            return np.exp(self._w) * (np.cos(norm) + v / norm * np.sin(norm))
        else:
            return np.exp(self._w)

    def ln(self):
        v = self.v
        norm = self.norm()
        vnorm = v.norm()
        if norm == 0:
            raise ValueError('Cannot take the logarithm of zero')
        elif vnorm == 0:
            return Quaternion(np.log(norm))
        return Quaternion(np.log(norm), v/vnorm*np.arccos(self._w/norm))

    def conjugate(self):
        return Quaternion(self._w, -self._x, -self._y, -self._z)

    conj = conjugate

    def norm(self):
        return np.sqrt(self._w * self._w + self._x * self._x +
                       self._y * self._y + self._z * self._z)

    def normalize(self):
        length = self.norm()
        self /= length
        return self

    def __get_w(self):
        return self._w

    def __set_w(self, val):
        if isinstance(val, numbers.Real):
            self._w = val
        else:
            raise TypeError()

    def __get_x(self):
        return self._x

    def __set_x(self, val):
        if isinstance(val, numbers.Real):
            self._x = val
        else:
            raise TypeError()

    def __get_y(self):
        return self._y

    def __set_y(self, val):
        if isinstance(val, numbers.Real):
            self._y = val
        else:
            raise TypeError()

    def __get_z(self):
        return self._z

    def __set_z(self, val):
        if isinstance(val, numbers.Real):
            self._z = val
        else:
            raise TypeError()

    def __get_v(self):
        return Quaternion(0, self._x, self._y, self._z)
        # return np.array([self._x, self._y, self._z])

    def __set_v(self, val):
        if isinstance(val, Quaternion):
            self._x = val._x
            self._y = val._y
            self._z = val._z
        elif isinstance(val, np.ndarray) and val.size == 3:
            self._x = val.flatten()[0]
            self._y = val.flatten()[1]
            self._z = val.flatten()[2]
        else:
            raise TypeError()

    def copy(self):
        return Quaternion(self._w, self._x, self._y, self._z)

    def __str__(self):
        return ('<{:.2f}, {:.2f}i, {:.2f}j, {:.2f}k>'
                .format(self._w, self._x, self._y, self._z))

    def __repr__(self):
        return ('Quaternion({:.2f}, {:.2f}, {:.2f}, {:.2f})'
                .format(self._w, self._x, self._y, self._z))

    w = property(__get_w, __set_w)
    v = property(__get_v, __set_w)
    x = property(__get_x, __set_x)
    y = property(__get_y, __set_y)
    z = property(__get_z, __set_z)


def rotate(vec, axis, angle):
    rot = Quaternion(np.cos(angle/2), *(axis * np.sin(angle/2))).normalize()
    v = Quaternion(0, *vec)
    return (rot * v * rot.conj()).normalize()
