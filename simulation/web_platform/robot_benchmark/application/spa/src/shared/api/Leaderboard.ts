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

export class Leaderboard<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * @description Return Общий Leaderboard
     *
     * @tags leaderboard
     * @name LeaderboardRetrieve
     * @request GET:/api/leaderboard/
     * @secure
     */
    leaderboardRetrieve = (params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/leaderboard/`,
            method: "GET",
            secure: true,
            ...params,
        });
    /**
     * @description Return Leaderboard определённой задачи
     *
     * @tags leaderboard
     * @name LeaderboardProblemRetrieve
     * @request GET:/api/leaderboard/problem/{problem_id}/
     * @secure
     */
    leaderboardProblemRetrieve = (problemId: number, params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/leaderboard/problem/${problemId}/`,
            method: "GET",
            secure: true,
            ...params,
        });
    /**
     * @description Return Leaderboard определённого турнира
     *
     * @tags leaderboard
     * @name LeaderboardTournamentRetrieve
     * @request GET:/api/leaderboard/tournament/{tournament_id}/
     * @secure
     */
    leaderboardTournamentRetrieve = (tournamentId: number, params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/leaderboard/tournament/${tournamentId}/`,
            method: "GET",
            secure: true,
            ...params,
        });
}
