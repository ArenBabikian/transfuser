
export CARLA_ROOT=/home/ubuntu/git/transfuser/carla
export WORK_DIR=/home/ubuntu/git/transfuser

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla

export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

# Things relevant to us
TIME=$(date +"%m%d-%H%M%S")

export SAVE_DIR=${WORK_DIR}/fse/res/${TIME}
mkdir $SAVE_DIR

# CONFIGURATION
export DATA_DIR=${WORK_DIR}/fse/config
export SCENARIOS=${DATA_DIR}/empty_scenarios.json
export ROUTES=${DATA_DIR}/Town05_2240_1_4_actors_all_scenarios.xml

export REPETITIONS=10 # IMPORTANT
export CHALLENGE_TRACK_CODENAME=SENSORS
export CHECKPOINT_ENDPOINT=${SAVE_DIR}/measurements.json
export TEAM_AGENT=${WORK_DIR}/team_code_transfuser/submission_agent.py
export TEAM_CONFIG=${WORK_DIR}/model_ckpt/models_2022/transfuser

export DEBUG_CHALLENGE=0
export RESUME=
export DATAGEN=0
export RECORD=${SAVE_DIR}

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
--timeout=60 \
