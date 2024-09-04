from calibrate import straight_move, rotate_move

from time import sleep

for _ in range(5):
    for i in range(4):
        print("moving straight")
        straight_move()
        print("waiting to rotate")
        sleep(2)
        print("rotating")
        rotate_move(sleep_s=0.725,mdir=(0,1))
        sleep(2)
    sleep(1)
