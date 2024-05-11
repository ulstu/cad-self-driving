import { useCallback, useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";
import { Tournament } from "../../shared/api/data-contracts";
import { Button, Card, Col, Row } from "antd";
import CreateTournamentModal from "./TournamentCreateModal";

export const TournamentList = () => {
  const [tournaments, setTournaments] = useState<Tournament[]>([]);
  const navigate = useNavigate();
  const [visible, setVisible] = useState(false);

  const token = useAuthStore((state) => state.token);

  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
    baseApiParams: {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    },
  };

  const apiClient = new apiClientClass(configMcc);

  const fetchTournaments = useCallback(() => {
    apiClient.Tournament.tournamentList()
      .then((response) => {
        setTournaments(response.data);
      })
      .catch((error) => console.error("Не удалось загрузить турниры:", error));
  }, []);

  useEffect(() => {
    fetchTournaments();
  }, [fetchTournaments]);

  const onCreate = (values: Tournament) => {
    apiClient.Tournament.tournamentCreate({
      ...values,
    }).then(() => {
      setVisible(false);
      fetchTournaments();
    });
  };

  return (
    <div>
      <h1>Открытые турниры</h1>
      <Button type="primary" onClick={() => setVisible(true)}>
        Создать турнир
      </Button>
      <CreateTournamentModal
        visible={visible}
        onCreate={onCreate}
        onCancel={() => setVisible(false)}
      />
      {tournaments && tournaments.length > 0 ? (
        <Row gutter={16} style={{ marginTop: 16 }}>
          {tournaments.map((tournament) => (
            <Col key={tournament.id} span={8} style={{ marginBottom: 16 }}>
              <Card
                title={tournament.name}
                bordered={true}
                hoverable
                onClick={() => navigate(`/tournaments/${tournament.id}`)}
              >
                <p>Описание: {tournament.description}</p>
                <p>Нажмите для просмотра деталей</p>
              </Card>
            </Col>
          ))}
        </Row>
      ) : (
        <p>Нет открытых турниров</p>
      )}
    </div>
  );
};

export default TournamentList;
