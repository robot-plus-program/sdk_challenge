import numpy as np

v2str = lambda v: str(v.dtype.name) + str(v.shape) if type(v).__name__ == "ndarray" else v

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def show_data(dat):
    print('Server received data: ')
    if not isinstance(dat, list):
        print(f'{v2str(dat)}')
        return

    for k, v in dat.items():
        print(f'{k}: {v2str(v)}')

def ins2byte(ins, con='_'):
    try:
        type_name = type(ins).__name__
        con_encoded = con.encode()
        if type_name in ['list', 'tuple', 'set']:
            op = eval(type_name)
            return str(type_name).encode() + con_encoded + str(op(ins2byte(el, con) for el in ins)).encode()
        if type_name == 'dict':
            return str(type_name).encode() + con_encoded + str({k: ins2byte(ins[k], con) for k in ins}).encode()

        if type_name != 'ndarray':
            return type_name.encode() + con_encoded + str(ins).encode()
        return (type_name.encode() + con_encoded + ins.dtype.name.encode() + con_encoded +
                str(ins.shape).encode() + con_encoded + ins.tobytes())
    except:
        return ins2byte(None)

def byte2ins(byteData, con='_'):
    try:
        con_encoded = con.encode()
        i = byteData.index(con_encoded)
        type_name, byteData = byteData[:i].decode(), byteData[i + 1:]

        if type_name in ['list', 'tuple', 'set']:
            op = eval(type_name)
            return op(byte2ins(el) for el in eval(byteData.decode()))
        if type_name == 'dict':
            decoded = eval(byteData.decode())
            return {k: byte2ins(decoded[k]) for k in decoded}

        if type_name != 'ndarray':
            op = eval(type_name)
            return op(byteData.decode())
        i = byteData.index(con_encoded)
        dtype, byteData = byteData[:i].decode(), byteData[i + 1:]
        i = byteData.index(con_encoded)
        shape, byteData = eval(byteData[:i].decode()), byteData[i + 1:]
        return np.frombuffer(byteData, dtype=dtype).reshape(shape)
    except:
        return None



def decorate_show_log(function):
    def wrapper(*args, **kwargs):
        print(f'==============={function.__name__.upper()}')
        try:
            print('running')
            ret = function(*args,**kwargs)
            print('finished')
        except:
            ret = None
            print(f'XXXXXXX Failed or no {function.__name__} method ...')
        return ret
    return wrapper

def decorate_all_class_method(decorator):
    def decorate(cls):
        for name, method in cls.__dict__.items():
            if callable(method):
                setattr(cls, name, decorator(method))
        return cls
    return decorate
