"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = __importDefault(require("express"));
const auth_route_1 = __importDefault(require("./auth/auth.route"));
const equipament_route_1 = __importDefault(require("./equipament/equipament.route"));
const app = (0, express_1.default)();
app.use(express_1.default.json());
app.use('/api', auth_route_1.default);
app.use('/api/equipament', equipament_route_1.default);
//index route
app.get('/', (req, res) => {
    res.json({
        message: 'Servidor Rodando'
    });
});
// Rotas inexistentes (404)
app.use((req, res) => {
    res.status(404).json({
        message: 'Rota não encontrada',
    });
});
// Tratamento global de erros
app.use((err, req, res, next) => {
    console.error(err);
    res.status(err.status || 500).json({
        message: err.message || 'Erro interno do servidor',
    });
});
exports.default = app;
