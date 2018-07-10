def main():
    a = 1
    b = 6 ** 2
    c = 6 // 2
    d = 6 / 4
    e = a * b
    print(a, b, c, d, e)

    s1 = "a"
    s2 = 'a'

    s3 = s1 + s2

    print(s1, s2, s3)

    l = [1, 'a']

    l.append(66)
    l[2] = 6
    print(l, l[0], l[1], len(l))

    di = {1: "abc", 2: "def"}
    print(di, di[1], di[2], di.get(3))

    tu = (3, 5, 6)

    for i in range(3, 10, 2):
        print(i)
    foo()
    foo2(3)

    myclass = Cls()
    myclass.foo3()
    Cls.foo3(myclass)


def foo():
    print("in foo")


def foo2(a):
    print("in foo", a)


class Cls:
    def __init__(self):
        print("in init")
        self.val = 6

    def foo3(self):
        print("foo3")


if __name__ == '__main__':
    main()
