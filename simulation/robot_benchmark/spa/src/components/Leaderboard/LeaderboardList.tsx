import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";

export const LeaderboardList = () => {
  const [tournaments, setTournaments] = useState<any[]>([]);
  const navigate = useNavigate();

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

  useEffect(() => {
    apiClient.Leaderboard.leaderboardRetrieve()
      .then((response) => {
        // Assuming the response data is the list of tournaments
        setTournaments(response.data);
      })
      .catch((error) => console.error('Failed to fetch tournaments:', error));
  }, []);

  return (
    <div>
      <h1>Leaderboard</h1>
    </div>
  );
};
