import { resolve } from "path";
import { generateApi } from "swagger-typescript-api";

generateApi({
  // eslint-disable-next-line no-undef
  input: resolve(process.cwd(), "../robotbenchmark/OpenAPI/API.yaml"),
  // eslint-disable-next-line no-undef
  output: resolve(process.cwd(), "./src/shared/api/"),
  name: "OpenApi.ts",
  prettier: {
    trailingComma: "all",
    tabWidth: 4,
    printWidth: 160,
  },
  primitiveTypeConstructs: (constructs) => ({
    ...constructs,
    string: {
      "date-time": "Date",
    },
  }),
  modular: true,
  moduleNameFirstTag: true,
});
