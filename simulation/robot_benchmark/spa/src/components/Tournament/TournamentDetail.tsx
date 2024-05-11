import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { Tournament, apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";
import { Row, Col, Card, List } from "antd";

export const TournamentDetail = () => {
  const params = useParams();
  const token = useAuthStore((state) => state.token);
  const [tournament, setTournament] = useState<Tournament>();

  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
    baseApiParams: {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    },
  };

  const apiClient = new apiClientClass(configMcc);

  useEffect(() => {
    if (params.id) {
      apiClient.Tournament.tournamentRetrieve(Number(params.id)).then((item) => setTournament(item.data));
    }
  }, [params.id]);

  if (tournament) {
    return (
      <Row gutter={16}>
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
              renderItem={(item) => <List.Item>Задача #{item}</List.Item>}
            />
          </Card>
        </Col>
        <Col span={12}>
          <Card title="Участники">
            <List
              bordered
              dataSource={tournament.users}
              renderItem={(user) => <List.Item>Участник #{user}</List.Item>}
            />
          </Card>
        </Col>
      </Row>
    );
  }
};
