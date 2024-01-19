# Simulator

Reads generated graph route network scenario from `graphgenerator` and runs various traffic conditions.

## How to use

1. Select graph scenario from `graphgenerator`, modify `\sim_input\conf.txt`
2. Run `flowopt_uni.py` to generate uniform schedule of allowed takeoffs per depot locations
- assumed seperate lanes for incoming/outgoing vehicles
- assumed uniform periodic schedule, e.g. every 5 seconds a flight can takeoff at depot 2 to depot 3
3. Run `create_data.py` to generate traffic scenario data: (icnt,ideptid,icustid,it)
- uses `customer_time.py`
- three distribution generators: exponential, uniform, constant
4. Run `main.py`, TODO

## Subsystems

1. agent model: `drone.py`, USER: `data_queue.py`
2. tbov model: `tbov.py`, USER `data_queue.py`
3. depot model: `data_queue.py`, NEED: `drone.py`, `tbov.py`
4. schedule algorthms: `network_schedular.py`

