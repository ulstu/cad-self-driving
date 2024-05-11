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

import { TokenObtainPair, TokenRefresh } from "./data-contracts";
import { ContentType, HttpClient, RequestParams } from "./http-client";

export class Token<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * @description Takes a set of user credentials and returns an access and refresh JSON web token pair to prove the authentication of those credentials.
     *
     * @tags token
     * @name TokenCreate
     * @request POST:/api/token/
     */
    tokenCreate = (data: TokenObtainPair, params: RequestParams = {}) =>
        this.request<TokenObtainPair, any>({
            path: `/api/token/`,
            method: "POST",
            body: data,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * @description Takes a refresh type JSON web token and returns an access type JSON web token if the refresh token is valid.
     *
     * @tags token
     * @name TokenRefreshCreate
     * @request POST:/api/token/refresh/
     */
    tokenRefreshCreate = (data: TokenRefresh, params: RequestParams = {}) =>
        this.request<TokenRefresh, any>({
            path: `/api/token/refresh/`,
            method: "POST",
            body: data,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
}
