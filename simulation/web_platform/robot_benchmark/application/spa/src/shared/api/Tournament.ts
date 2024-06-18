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

import { PatchedTournament, Tournament } from "./data-contracts";
import { ContentType, HttpClient, RequestParams } from "./http-client";

export class Tournament<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * @description ViewSet Соревнования
     *
     * @tags tournament
     * @name TournamentList
     * @summary Получение списка соревнований
     * @request GET:/api/tournament/
     * @secure
     */
    tournamentList = (
        query?: {
            /** Which field to use when ordering the results. */
            ordering?: string;
        },
        params: RequestParams = {},
    ) =>
        this.request<Tournament[], any>({
            path: `/api/tournament/`,
            method: "GET",
            query: query,
            secure: true,
            format: "json",
            ...params,
        });
    /**
     * @description ViewSet Соревнования
     *
     * @tags tournament
     * @name TournamentCreate
     * @summary Создание соревнования
     * @request POST:/api/tournament/
     * @secure
     */
    tournamentCreate = (data: Tournament, params: RequestParams = {}) =>
        this.request<Tournament, any>({
            path: `/api/tournament/`,
            method: "POST",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * @description ViewSet Соревнования
     *
     * @tags tournament
     * @name TournamentRetrieve
     * @summary Детальная информация о соревновании
     * @request GET:/api/tournament/{id}/
     * @secure
     */
    tournamentRetrieve = (id: number, params: RequestParams = {}) =>
        this.request<Tournament, any>({
            path: `/api/tournament/${id}/`,
            method: "GET",
            secure: true,
            format: "json",
            ...params,
        });
    /**
     * @description ViewSet Соревнования
     *
     * @tags tournament
     * @name TournamentUpdate
     * @summary Обновление данных о соревновании
     * @request PUT:/api/tournament/{id}/
     * @secure
     */
    tournamentUpdate = (id: number, data: Tournament, params: RequestParams = {}) =>
        this.request<Tournament, any>({
            path: `/api/tournament/${id}/`,
            method: "PUT",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * @description ViewSet Соревнования
     *
     * @tags tournament
     * @name TournamentPartialUpdate
     * @summary Обновление с необязательными полями соревнования
     * @request PATCH:/api/tournament/{id}/
     * @secure
     */
    tournamentPartialUpdate = (id: number, data: PatchedTournament, params: RequestParams = {}) =>
        this.request<Tournament, any>({
            path: `/api/tournament/${id}/`,
            method: "PATCH",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * @description ViewSet Соревнования
     *
     * @tags tournament
     * @name TournamentDestroy
     * @summary Удаление соревнования
     * @request DELETE:/api/tournament/{id}/
     * @secure
     */
    tournamentDestroy = (id: number, params: RequestParams = {}) =>
        this.request<Tournament, any>({
            path: `/api/tournament/${id}/`,
            method: "DELETE",
            secure: true,
            format: "json",
            ...params,
        });
}
