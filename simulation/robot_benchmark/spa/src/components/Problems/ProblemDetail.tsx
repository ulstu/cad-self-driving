import { Button, Col, Row, Card, Tabs, TabPaneProps } from "antd";
import "./ProblemDetail.css"; // Basic CSS for additional styling if needed
import { useEffect, useState } from "react";
import { Problem, apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";
import { useParams } from "react-router-dom";

export interface Tab extends Omit<TabPaneProps, 'tab'> {
  key: string;
  label: React.ReactNode;
}

export const ProblemDetail = () => {
  const [problem, setProblem] = useState<Problem>();
  const token = useAuthStore((state) => state.token);
  const params = useParams();

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
      apiClient.Problems.problemRetrieve(Number(params.id)).then((res) => {
        setProblem(res.data.problem);
      });
    }
  }, [params.id]);

  const items: Tab[] = [{key: '1', label: 'VS Code', children: <iframe src="http://localhost:10002" style={{height: '100%', width: '100%'}} />, style: {height: '100%'} },
   {key: '2', label: 'Webots', children: <iframe src="http://localhost:10003/index.html" style={{height: '100%', width: '100%'}} />, style: {height: '100%'}},
   {key: '3', label: 'Редактор карты', children: <iframe src="http://localhost:10001" style={{height: '100%', width: '100%'}} />, style: {height: '100%'}}]

  return (
    <div className="problem-detail-container">
      {problem ? (
        <Tabs style={{height: '100vh'}} items={items} />
      ) : (
        <h1>Ничего нет</h1>
      )}
    </div>
  );
};

export default ProblemDetail;
