from ketisdk.gui.gui import GUI, GuiModule
import argparse


def get_cfg():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_client', default=2, type=int, help='number of TCP clients')
    parser.add_argument('--sensors', default='realsense', type=str, help='sensors in use')
    parser.add_argument('--title', default='Demo TCP clients', type=str, help='title')
    parser.add_argument('--hosts', default='localhost:5000,localhost:5001', type=str, help='title')
    return parser.parse_args()


def main(cfg):
    modules = []
    if 'realsense' in cfg.sensors:
        from ketisdk.sensor.realsense_sensor import get_realsense_modules
        modules += get_realsense_modules()

    hosts = cfg.hosts.replace(' ', '').split(',')
    for i in range(cfg.num_client):
        host, port = hosts[i].split(':')
        modules.append(GuiModule(None, type='tcp_client', name=f'Client{i}', category='tcp_client',
                                 host=host, port=int(port)))

    GUI(title=cfg.title, modules=modules)


if __name__ == '__main__':
    cfg = get_cfg()
    main(cfg)
