"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.equipamentController = void 0;
const equipament_service_1 = require("./equipament.service");
function findAll(req, res) {
    return __awaiter(this, void 0, void 0, function* () {
        try {
            const filters = req.validatedQuery; // já validado pelo middleware
            const equipaments = yield equipament_service_1.equipamentService.findAll(filters);
            res.json(equipaments);
        }
        catch (error) {
            res.status(500).json({
                message: "Erro ao buscar equipamentos",
                error
            });
        }
    });
}
function findById(req, res) {
    return __awaiter(this, void 0, void 0, function* () {
        const id = req.params.id;
        try {
            const parsedId = Number.parseInt(id);
            const equipament = yield equipament_service_1.equipamentService.findById(parsedId);
            if (!equipament) {
                res.status(404).json({
                    message: "Equipamento inexistente"
                });
            }
            res.json(equipament);
        }
        catch (error) {
            res.status(500).json({
                message: "Erro ao buscar equipamento por id",
                error
            });
        }
    });
}
function create(req, res) {
    return __awaiter(this, void 0, void 0, function* () {
        const { name, type, location, rfid, responsible } = req.body;
        try {
            const checkEquipament = yield equipament_service_1.equipamentService.findByRFID(rfid);
            if (checkEquipament) {
                res.status(409).json({ message: "Já existe um equipamento com esse rfid" });
            }
            else {
                const equipament = yield equipament_service_1.equipamentService.create(name, type, location, rfid, responsible);
                res.status(201).json(equipament);
            }
        }
        catch (erro) {
            res.status(500).json({
                message: "Erro ao cadastrar equipamento",
                erro
            });
        }
    });
}
function alertOutRange(req, res) {
    return __awaiter(this, void 0, void 0, function* () {
        const { rfid } = req.body;
        try {
            yield equipament_service_1.equipamentService.alertOutRange(rfid);
            res.json({ message: `Equipamento com rfid: ${rfid} marcado como fora do alcance e notificação enviada para todos os usuarios (se aplicável).` });
        }
        catch (error) {
            res.status(500).json({
                message: "Erro ao fazer alerta",
                error
            });
        }
    });
}
exports.equipamentController = {
    findAll,
    findById,
    create,
    alertOutRange
};
