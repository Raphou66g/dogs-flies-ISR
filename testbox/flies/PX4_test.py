import threading
from time import sleep

a = [True, False]

def dodo(time:int, pos, val):
    sleep(time)
    print(a)
    if pos == -1:
        a.pop(0)
    else:
        a[pos] = val
    print(f"new : {a}")

def main():
    th1 = threading.Thread(target=dodo, args=(5,-1,False))
    th2 = threading.Thread(target=dodo, args=(20,0,True))

    th1.start()
    th2.start()

    z = True
    while z:
        sleep(3)
        t1 = th1.is_alive()
        t2 = th2.is_alive()
        print(f"th1 : {t1} / th2 : {t2}")
        z = t1 or t2

    print("END")

if __name__ == '__main__':
    main()