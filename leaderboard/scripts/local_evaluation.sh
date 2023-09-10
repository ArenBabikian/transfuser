# export CARLA_ROOT=${1:-/home/kchitta/Documents/CARLA_0.9.10.1}
# export WORK_DIR=${2:-/home/kchitta/Documents/transfuser}

export CARLA_ROOT=/home/ubuntu/aren/transfuser/carla
export WORK_DIR=/home/ubuntu/aren/transfuser

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
# BELOW is commented because we wanna use the carla API downloaded by pip
# export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
# export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}
export PYTHONPATH="${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export SAVE_DIR=${LEADERBOARD_ROOT}/aren/res
export DATA_DIR=${LEADERBOARD_ROOT}/aren/data2
# Modify below
export SCENARIOS=${DATA_DIR}/empty_scenarios.json
export ROUTES=${DATA_DIR}/town02_r1.xml

export REPETITIONS=1
export CHALLENGE_TRACK_CODENAME=SENSORS
TIME=$(date +"%m%d-%H%M%S")
export CHECKPOINT_ENDPOINT=${SAVE_DIR}/${TIME}.json
export TEAM_AGENT=${WORK_DIR}/team_code_transfuser/submission_agent.py
export TEAM_CONFIG=${WORK_DIR}/model_ckpt/models_2022/transfuser

export DEBUG_CHALLENGE=1
# 0 does not work, needs to be emmpty
export RESUME=
export DATAGEN=0
export RECORD=${SAVE_DIR}/${TIME}.log
# export RECORD=

python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator_local.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--resume=${RESUME} \
--record=${RECORD} \
--timeout=30 \
