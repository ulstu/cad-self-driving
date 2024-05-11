/* eslint-disable */
/* tslint:disable */
/*
 * ---------------------------------------------------------------
 * ## THIS FILE WAS GENERATED VIA SWAGGER-TYPESCRIPT-API        ##
 * ##                                                           ##
 * ## AUTHOR: acacode                                           ##
 * ## SOURCE: https://github.com/acacode/swagger-typescript-api ##
 * ---------------------------------------------------------------
 */

import { HttpClient, RequestParams } from "./http-client";

export class Api<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * @description Return Общий Leaderboard
     *
     * @tags api
     * @name ApiLeaderboardRetrieve
     * @request GET:/api/leaderboard/
     * @secure
     */
    apiLeaderboardRetrieve = (params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/leaderboard/`,
            method: "GET",
            secure: true,
            ...params,
        });
    /**
     * @description Return Leaderboard определённой задачи
     *
     * @tags api
     * @name ApiLeaderboardProblemRetrieve
     * @request GET:/api/leaderboard/problem/{problem_id}/
     * @secure
     */
    apiLeaderboardProblemRetrieve = (problemId: number, params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/leaderboard/problem/${problemId}/`,
            method: "GET",
            secure: true,
            ...params,
        });
    /**
     * @description Return Leaderboard определённого турнира
     *
     * @tags api
     * @name ApiLeaderboardTournamentRetrieve
     * @request GET:/api/leaderboard/tournament/{tournament_id}/
     * @secure
     */
    apiLeaderboardTournamentRetrieve = (tournamentId: number, params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/leaderboard/tournament/${tournamentId}/`,
            method: "GET",
            secure: true,
            ...params,
        });
}
