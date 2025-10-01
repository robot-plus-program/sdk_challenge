class PICK:
    GPOINT = 0
    GRIP_TYPE = 1
    FLAG = 2
    X = 3
    Y = 4
    Z = 5
    RX = 6
    RY = 7
    RZ = 8
    ROT_TYPE = 9
    GRIP_THETA = 10
    OBJ_WIDTH = 11

class GRIP_TYPE:
    SUCTION = 0
    TWO_FINGER_GRIP = 1
    THREE_FINGER_GRIP = 2
    INNER_FINGER_GRIP = 3

PICK_MSG = [
    PICK.GPOINT,
    PICK.GRIP_TYPE,
    PICK.FLAG,
    PICK.X,
    PICK.Y,
    PICK.Z,
    PICK.RX,
    PICK.RY,
    PICK.RZ,
    PICK.ROT_TYPE   ,
    PICK.GRIP_THETA ,
    PICK.OBJ_WIDTH]

if __name__ == '__main__':
    from ketisdk.base.protocols import PICK
    from ketisdk.base.protocols import GRIP_TYPE
    from ketisdk.base.protocols import PICK_MSG
    
    PICK_MSG[PICK.GRIP_TYPE] = GRIP_TYPE.INNER_FINGER_GRIP
    PICK_MSG[PICK.X] = 100
    PICK_MSG[PICK.Y] = 100
    PICK_MSG[PICK.Z] = 100
    PICK_MSG[PICK.GRIP_THETA] = 100
    
    SEND(PICK_MSG)
    RECEIVE(PICK_MSG)
    
    
