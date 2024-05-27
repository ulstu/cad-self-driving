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

import { PatchedProblem, Problem } from "./data-contracts";
import { ContentType, HttpClient, RequestParams } from "./http-client";

export class Problems<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * No description
     *
     * @tags problems
     * @name ProblemList
     * @summary Получение списка задач
     * @request GET:/api/problem/
     * @secure
     */
    problemList = (
        query?: {
            /** Which field to use when ordering the results. */
            ordering?: string;
            title?: string;
        },
        params: RequestParams = {},
    ) =>
        this.request<Problem[], any>({
            path: `/api/problem/`,
            method: "GET",
            query: query,
            secure: true,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags problems
     * @name ProblemCreate
     * @summary Создание задачи
     * @request POST:/api/problem/
     * @secure
     */
    problemCreate = (data: Problem, params: RequestParams = {}) =>
        this.request<Problem, any>({
            path: `/api/problem/`,
            method: "POST",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags problems
     * @name ProblemRetrieve
     * @summary Детальная информация о задаче
     * @request GET:/api/problem/{id}/
     * @secure
     */
    problemRetrieve = (id: number, params: RequestParams = {}) =>
        this.request<Problem, any>({
            path: `/api/problem/${id}/`,
            method: "GET",
            secure: true,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags problems
     * @name ProblemUpdate
     * @summary Обновление данных о задачи
     * @request PUT:/api/problem/{id}/
     * @secure
     */
    problemUpdate = (id: number, data: Problem, params: RequestParams = {}) =>
        this.request<Problem, any>({
            path: `/api/problem/${id}/`,
            method: "PUT",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags problems
     * @name ProblemPartialUpdate
     * @summary Обновление с необязательными полями задачи
     * @request PATCH:/api/problem/{id}/
     * @secure
     */
    problemPartialUpdate = (id: number, data: PatchedProblem, params: RequestParams = {}) =>
        this.request<Problem, any>({
            path: `/api/problem/${id}/`,
            method: "PATCH",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags problems
     * @name ProblemDestroy
     * @summary Удаление задачи
     * @request DELETE:/api/problem/{id}/
     * @secure
     */
    problemDestroy = (id: number, params: RequestParams = {}) =>
        this.request<Problem, any>({
            path: `/api/problem/${id}/`,
            method: "DELETE",
            secure: true,
            format: "json",
            ...params,
        });
}
