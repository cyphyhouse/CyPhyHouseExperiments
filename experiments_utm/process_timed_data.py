import matplotlib.pyplot as plt
import pickle 

def process_data():
    fn = './trajectories/backup/drone0_client_state_received'
    drone0_client_sr = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone0_client_sr.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone1_client_state_received'
    drone1_client_sr = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone1_client_sr.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone0_client_state_handled'
    drone0_client_sh = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone0_client_sh.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone1_client_state_handled'
    drone1_client_sh = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone1_client_sh.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/server_state_received'
    server_sr = []
    with open(fn,'rb') as f:
        while True:
            try:
                server_sr.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/server_acas_handled'
    server_ah = []
    with open(fn, 'rb') as f:
        while True:
            try:
                server_ah.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone0_client_acas_received'
    drone0_client_ar = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone0_client_ar.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone1_client_acas_received'
    drone1_client_ar = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone1_client_ar.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone0_client_acas_handled'
    drone0_client_ah = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone0_client_ah.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone1_client_acas_handled'
    drone1_client_ah = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone1_client_ah.append(pickle.load(f))
            except EOFError:
                break
        
    fn = './trajectories/backup/drone0_agent_acas_received'
    drone0_agent_ar = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone0_agent_ar.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone1_agent_acas_received'
    drone1_agent_ar = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone1_agent_ar.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone0_agent_acas_handled'
    drone0_agent_ah = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone0_agent_ah.append(pickle.load(f))
            except EOFError:
                break

    fn = './trajectories/backup/drone1_agent_acas_handled'
    drone1_agent_ah = []
    with open(fn,'rb') as f:
        while 1:
            try:
                drone1_agent_ah.append(pickle.load(f))
            except EOFError:
                break

    # all_data = [
    #     drone0_client_sr,   # 0
    #     drone0_client_sh,   # 1
    #     server_sr,          # 2
    #     server_ah,          # 3
    #     drone0_client_ar,   # 4
    #     drone0_client_ah,   # 5
    #     drone0_agent_ar,    # 6
    #     drone0_agent_ah,    # 7
    # ]


    all_data = [
        drone0_client_sr,   # 0
        drone1_client_sr,   # 1
        drone0_client_sh,   # 2
        drone1_client_sh,   # 3  
        server_sr,          # 4
        server_ah,          # 5
        drone0_client_ar,   # 6
        drone1_client_ar,   # 7
        drone0_client_ah,   # 8
        drone1_client_ah,   # 9 
        drone0_agent_ar,    # 10
        drone1_agent_ar,    # 11
        drone0_agent_ah,    # 12
        drone1_agent_ah     # 13
    ]

    for i in range(len(all_data)):
        data = all_data[i]
        plot_data_x = []
        plot_data_y = []
        for data_point in data:
            plot_data_x.append(data_point[0])
            plot_data_y.append(i)
            if i == 7:
                plt.plot([data_point[0], data_point[0]], [-1,8],'r')

        plt.plot(plot_data_x, plot_data_y,'b.')

    plt.show()

if __name__ == "__main__":
    process_data()