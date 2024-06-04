import { useCallback, useEffect, useState } from "react";
import { useNavigate, useParams } from "react-router-dom";
import { ProblemUser, Tournament, apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";
import { Row, Col, Card, List, Button, Space, Flex, FloatButton } from "antd";
import { EditOutlined, PlayCircleFilled } from "@ant-design/icons";
import { useProblemsStore } from "../../store/useProblemsStore";
import "./TournamentDetail.scss";
import { TournamentEdit } from "./TournamentEdit";

export const TournamentDetail = () => {
  const params = useParams();
  const { token, userId } = useAuthStore();
  const setLevelData = useProblemsStore((state) => state.setData);
  const [tournament, setTournament] = useState<Tournament>();
  const [issuesInWork, setIssuesInWork] = useState<ProblemUser[]>();
  const [showTournamentEdit, setShowTournamentEdit] = useState<
    number | boolean
  >(false);

  const navigate = useNavigate();

  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
    baseApiParams: {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    },
  };

  const apiClient = new apiClientClass(configMcc);

  const startUserProblem = useCallback((userId: number, problemId: number) => {
    apiClient.UsersProblem.usersProblemCreate({
      user: userId,
      problem: problemId,
      points: 1231313,
      is_completed: false,
    }).then(({ data }) => {
      setLevelData(
        data.vs_port,
        data.webots_stream_port,
        data.problem,
        data.robot_panel_port
      );
      navigate(`/problems/${data.problem}`);
    });
  }, []);

  const continueUserProblem = useCallback((problem: number) => {
    apiClient.UsersProblem.usersProblemRetrieve(problem).then(({ data }) => {
      setLevelData(
        data.vs_port,
        data.webots_stream_port,
        data.problem,
        data.robot_panel_port
      );
      navigate(`/problems/${data.problem}`);
    });
  }, []);

  const findIssue = (id: number) => {
    if (issuesInWork)
      for (let i = 0; i < issuesInWork.length; i++) {
        if (issuesInWork[i].id === id) {
          return issuesInWork[i].id;
        } else return false;
      }
  };

  useEffect(() => {
    if (params.id) {
      apiClient.Tournament.tournamentRetrieve(Number(params.id)).then(
        (item) => {
          setTournament(item.data);
          console.log(item.data);
        }
      );
      apiClient.UsersProblem.usersProblemList({
        tournament_id: Number(params.id),
      }).then(({ data }) => setIssuesInWork(data));
    }
  }, [params.id]);

  const editData = useCallback((data: Tournament) => {
    apiClient.Tournament.tournamentUpdate(params.id, data).then(({ data }) =>
      setTournament(data)
    );
  }, []);

  if (tournament) {
    return (
      <Row gutter={16}>
        <FloatButton
          shape="square"
          tooltip={<>Редактировать соревнование</>}
          type="primary"
          style={{ right: 42 }}
          onClick={() => setShowTournamentEdit(Number(params.id))}
          icon={<EditOutlined />}
        />
        <TournamentEdit
          visible={showTournamentEdit}
          onEdit={editData}
          data={tournament}
          onCancel={() => setShowTournamentEdit(false)}
        />
        <Col span={12}>
          <Card title="Информация о соревновании">
            <p>
              <strong>Название:</strong> {tournament?.name}
            </p>
            <p>
              <strong>Описание:</strong> {tournament?.description}
            </p>
            <p>
              <strong>Начало:</strong>{" "}
              {new Date(tournament.date_start).toLocaleString()}
            </p>
            <p>
              <strong>Окончание:</strong>{" "}
              {new Date(tournament.date_end).toLocaleString()}
            </p>
          </Card>
          <Card title="Задачи" style={{ marginTop: "20px" }}>
            <List
              bordered
              dataSource={tournament.problems}
              renderItem={(item) => (
                <List.Item>
                  <Flex
                    justify="space-between"
                    align="center"
                    style={{ width: "100%" }}
                  >
                    {findIssue(item) ? (
                      <>
                        Задача #{item}
                        <Button
                          type="default"
                          onClick={() => continueUserProblem(findIssue(item))}
                          icon={<PlayCircleFilled />}
                          className="continue-btn"
                        >
                          Продолжить выполнение задачи
                        </Button>
                      </>
                    ) : (
                      <>
                        Задача #{item}
                        <Button
                          type="primary"
                          onClick={() => startUserProblem(userId, item)}
                          icon={<PlayCircleFilled />}
                        >
                          Запустить задачу
                        </Button>
                      </>
                    )}
                  </Flex>
                </List.Item>
              )}
            />
          </Card>
        </Col>
        <Col span={12}>
          <Card title="Участники">
            <List
              bordered
              dataSource={tournament.users}
              renderItem={(user) => (
                <List.Item>Участник #{user.first_name}</List.Item>
              )}
            />
          </Card>
        </Col>
      </Row>
    );
  }
};
