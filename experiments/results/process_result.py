from yaml import safe_load


def gen_row(res_list):
    # Overall simulation time
    sim_time = max(elem['t_end'] for elem in res_list) - min(elem['t_start'] for elem in res_list)

    # Response time. Only count agents
    resp_time_list = [elem['t_end'] - elem['t_start'] for elem in res_list if elem['name'].startswith("Agent")]
    num_agents = len(resp_time_list)
    max_resp_time = max(resp_time_list)
    avg_resp_time = sum(resp_time_list) / len(resp_time_list)

    num_req = sum(elem.get('Req', 0) for elem in res_list)
    avg_req_region = sum(elem.get('R(Req)', 0) for elem in res_list) / num_req
    max_num_req_per_agent = max(elem.get('Req', 0) for elem in res_list)
    avg_num_req_per_agent = num_req / num_agents

    num_qm = sum(elem.get('Qm', 0) for elem in res_list)
    avg_qm_region = sum(elem.get('R(Qm)', 0) for elem in res_list) / num_qm

    air_mgr = [elem for elem in res_list if elem['name'] == "AirspaceManager"][0]
    num_qe = air_mgr.get('Qe', 0)
    avg_qe_region = air_mgr.get('R(Qe)', 0) / num_qe
    avg_qe_per_sec = num_qe / (air_mgr['t_end'] - air_mgr['t_start'])

    return (num_agents, sim_time, max_resp_time, avg_resp_time, num_req, avg_req_region,
            max_num_req_per_agent, avg_num_req_per_agent, num_qe, avg_qe_region, avg_qe_per_sec)


if __name__ == "__main__":
    with open('busy_corridor.yaml', 'r') as f:
        ret_dict = safe_load(f)
        sorted_res = sorted(iter(ret_dict.values()), key=len)
        for ret_list in sorted_res:
            print("%d &  %.2f &  %.2f &  %.2f &  %d &  %.2f &  %d &  %.2f &  %d &  %.2f & %.2f"
                  % gen_row(ret_list))
