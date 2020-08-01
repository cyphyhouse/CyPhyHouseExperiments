from collections import defaultdict
from yaml import safe_load

import numpy as np


def gen_row(res_list):
    # Overall simulation time
    sim_time = max(elem['t_end'] for elem in res_list) - min(elem['t_start'] for elem in res_list)

    # Response time. Only count agents
    resp_time_list = [elem['t_end'] - elem['t_start'] for elem in res_list if elem['name'].startswith("Agent")]
    num_agents = len(resp_time_list)
    max_resp_time = max(resp_time_list)
    avg_resp_time = sum(resp_time_list) / len(resp_time_list)

    num_req = sum(elem.get('Req', 0) for elem in res_list)
    avg_req_sec = num_req / max_resp_time
    avg_req_region = sum(elem.get('R(Req)', 0) for elem in res_list) / num_req

    num_req_per_agent_sec = [elem.get('Req', 0) / (elem['t_end'] - elem['t_start']) for elem in res_list]
    max_num_req_per_agent_sec = max(num_req_per_agent_sec)
    avg_num_req_per_agent_sec = sum(num_req_per_agent_sec) / num_agents

    num_qm = sum(elem.get('Qm', 0) for elem in res_list)
    avg_qm_region = sum(elem.get('R(Qm)', 0) for elem in res_list) / num_qm

    agent_list = [elem for elem in res_list if elem['name'].startswith("Agent")]
    num_fails = sum(elem.get('fail', 0) for elem in agent_list)
    fail_percent_list = [elem.get('fail', 0) * elem['Req'] / elem['R(Req)'] for elem in agent_list]
    avg_fail_percent_per_agent = 100 * sum(fail_percent_list) / num_agents

    air_mgr = [elem for elem in res_list if elem['name'] == "AirspaceManager"][0]
    num_qe = air_mgr.get('Qe', 0)
    avg_qe_per_sec = num_qe / (air_mgr['t_end'] - air_mgr['t_start'])
    avg_qe_region_per_sec = air_mgr.get('R(Qe)', 0) / (air_mgr['t_end'] - air_mgr['t_start'])

    return (num_agents, sim_time, max_resp_time, avg_resp_time,
            num_req, max_num_req_per_agent_sec, avg_num_req_per_agent_sec,
            avg_fail_percent_per_agent,
            avg_qe_per_sec, avg_qe_region_per_sec)


if __name__ == "__main__":
    with open('aggressive/city_sim.yaml', 'r') as f:
        ret_dict = safe_load(f)
        sorted_res = sorted(ret_dict.values(), key=len)

        print(f.name)
        row_list = []
        for ret_list in sorted_res:
            row = gen_row(ret_list)
            row_list.append(row)

        agt_num_dict = defaultdict(list)
        for row in row_list:
            agt_num_dict[row[0]].append(np.array(row))

        print([(n, len(v)) for n, v in sorted(agt_num_dict.items())])
        for n, v in sorted(agt_num_dict.items()):
            row = np.mean(v, axis=0)
            # Gen columns for response time
            print(row[0], row[2], row[3], row[2] - row[3])
            # Gen columns for Qe
            print(row[0], row[-2], row[-1])
            # Gen columns for sim time, #rect, fail rate
            print(row[1], row[-1], row[-3])


