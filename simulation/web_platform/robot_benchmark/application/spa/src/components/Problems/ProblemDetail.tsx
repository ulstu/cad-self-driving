import { Tabs, TabPaneProps, Spin } from "antd";
import "./ProblemDetail.css"; // Basic CSS for additional styling if needed
import { useEffect, useState } from "react";
import { Problem, apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";
import { useParams } from "react-router-dom";
import { useProblemsStore } from "../../store/useProblemsStore";

export interface Tab extends Omit<TabPaneProps, 'tab'> {
  key: string;
  label: React.ReactNode;
}

export const ProblemDetail = () => {
  const [problem, setProblem] = useState<Problem>();
  const token = useAuthStore((state) => state.token);
  const {vs_code, robot_panel_port, webots_stream_port} = useProblemsStore()
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
        setProblem(res.data);
      });
    }
  }, [params.id]);

  const items: Tab[] = [{key: '1', label: 'VS Code', children: <iframe src={`http://localhost:${vs_code}`} style={{height: '100%', width: '100%'}} />, style: {height: '100%'} },
   {key: '2', label: 'Webots', children: <iframe src={`http://localhost:${webots_stream_port}/index.html`} style={{height: '100%', width: '100%'}} />, style: {height: '100%'}},
   {key: '3', label: 'Редактор карты', children: <iframe src={`http://localhost:${robot_panel_port}`} style={{height: '100%', width: '100%'}} />, style: {height: '100%'}}]

  return (
    <div className="problem-detail-container">
      {problem ? (
        <Tabs style={{height: '100vh'}} items={items} />
      ) : (
        <Spin size={'default'} />
      )}
    </div>
  );
};

export default ProblemDetail;
