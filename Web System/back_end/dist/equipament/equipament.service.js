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
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.equipamentService = void 0;
const prisma_1 = __importDefault(require("../prisma-client/prisma"));
const sendEmail_1 = require("../utils/sendEmail");
function findAll(filters) {
    return __awaiter(this, void 0, void 0, function* () {
        const { whereName, type, location, responsible, inRange, page = 1, limit = 10, } = filters;
        const [data, total] = yield Promise.all([
            prisma_1.default.equipment.findMany({
                where: Object.assign(Object.assign(Object.assign(Object.assign(Object.assign({}, (whereName && {
                    name: {
                        contains: whereName,
                        mode: 'insensitive',
                    },
                })), (type && {
                    type: {
                        contains: type,
                        mode: 'insensitive',
                    },
                })), (location && {
                    location: {
                        contains: location,
                        mode: 'insensitive',
                    },
                })), (responsible && {
                    responsible: {
                        contains: responsible,
                        mode: 'insensitive',
                    },
                })), (typeof inRange === 'boolean' && {
                    isInRange: inRange,
                })),
                skip: (page - 1) * limit,
                take: limit,
            }),
            prisma_1.default.equipment.count({
                where: Object.assign(Object.assign(Object.assign(Object.assign(Object.assign({}, (whereName && {
                    name: {
                        contains: whereName,
                        mode: 'insensitive',
                    },
                })), (type && {
                    type: {
                        contains: type,
                        mode: 'insensitive',
                    },
                })), (location && {
                    location: {
                        contains: location,
                        mode: 'insensitive',
                    },
                })), (responsible && {
                    responsible: {
                        contains: responsible,
                        mode: 'insensitive',
                    },
                })), (typeof inRange === 'boolean' && {
                    isInRange: inRange,
                }))
            }),
        ]);
        return {
            data,
            total,
            page,
            limit,
            totalPages: Math.ceil(total / limit),
        };
    });
}
function findById(id) {
    return __awaiter(this, void 0, void 0, function* () {
        const equipament = yield prisma_1.default.equipment.findUnique({
            where: {
                id: id
            }
        });
        return equipament;
    });
}
function findByRFID(rfid) {
    return __awaiter(this, void 0, void 0, function* () {
        const equipament = yield prisma_1.default.equipment.findUnique({
            where: {
                rfid: rfid
            }
        });
        return equipament;
    });
}
function create(name, type, location, rfid, responsible) {
    return __awaiter(this, void 0, void 0, function* () {
        const equipament = yield prisma_1.default.equipment.create({
            data: {
                name: name,
                type: type,
                location: location,
                rfid: rfid,
                responsible: responsible
            }
        });
        return equipament;
    });
}
function alertOutRange(rfid) {
    return __awaiter(this, void 0, void 0, function* () {
        const equipment = yield prisma_1.default.equipment.findUnique({ where: { rfid } });
        if (!equipment) {
            return equipment;
        }
        // Marcar como fora do alcance
        yield prisma_1.default.equipment.update({
            where: { rfid },
            data: { isInRange: false },
        });
        // send email
        yield (0, sendEmail_1.sendAlertEmail)("damiao28.contato@gmail.com", equipment.name);
    });
}
exports.equipamentService = {
    findAll,
    findById,
    findByRFID,
    create,
    alertOutRange
};
