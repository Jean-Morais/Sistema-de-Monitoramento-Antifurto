import { Request, Response } from "express";
import { equipamentService } from "./equipament.service";
import { EquipmentFilters } from "../schema/equipament.schema";


async function findAll(req: Request, res: Response) {
    try {
        const filters = req.validatedQuery as EquipmentFilters; // já validado pelo middleware
        const equipaments = await equipamentService.findAll(filters);
        res.json(equipaments);
    } catch (error) {
        res.status(500).json({
            message: "Erro ao buscar equipamentos",
            error
        })
    }
}

async function findById(req: Request, res: Response) {
    const id = req.params.id;
    try {
        const parsedId = Number.parseInt(id);
        const equipament = await equipamentService.findById(parsedId);
        if (!equipament) {
            res.status(404).json({
                message: "Equipamento inexistente"
            })
        }
        res.json(equipament);
    } catch (error) {
        res.status(500).json({
            message: "Erro ao buscar equipamento por id",
            error
        });
    }
}

async function create(req: Request, res: Response) {
    const { name, type, location, rfid, responsible } = req.body;
    try {
        const checkEquipament = await equipamentService.findByRFID(rfid);
        if (checkEquipament) {
            res.status(409).json({ message: "Já existe um equipamento com esse rfid" })
        } else {
            const equipament = await equipamentService.create(name, type, location, rfid, responsible);
            res.status(201).json(equipament);
        }
    } catch (erro) {
        res.status(500).json({
            message: "Erro ao cadastrar equipamento",
            erro
        })
    }
}

async function updateByRFID(req: Request, res: Response) {
    const rfid = req.params.rfid;
    const {name, type, location, responsible} = req.body;
    try {
        const equipament = await equipamentService.updateByRFID(rfid,name, type, location, responsible);
        if (!equipament) {
            res.status(404).json({
                message : "Equipamento não encontrado"
            });
        }  
        res.json(equipament)
    } catch(erro) {
        res.status(500).json({
            message: "Erro ao atualizar equipamento",
            erro
        })
    }
}

async function deleteByRFID(req: Request, res: Response) {
    const rfid = req.params.rfid;
    try {
        const equipament = await equipamentService.deleteByRFID(rfid);
        if (!equipament) {
            res.status(404).json({
                message : "Equipamento não encontrado"
            });
        }  
        res.json(equipament);
    } catch (erro) {
        res.status(500).json({
            message: "Erro ao deletar equipamento",
            erro
        })
    }
}

async function alertOutRange(req: Request, res: Response) {
    const { rfid } = req.body;
    try {
        await equipamentService.alertOutRange(rfid);
        res.json({ message: `Equipamento com rfid: ${rfid} marcado como fora do alcance e notificação enviada para todos os usuarios (se aplicável).` });
    } catch (error) {
        res.status(500).json({
            message: "Erro ao fazer alerta",
            error
        })
    }
}

async function changeStatus(req: Request, res: Response) {
    const rfid = req.params.rfid;
    try {
        const equipament = await equipamentService.changeStatus(rfid);
        if(!equipament) {
            res.status(404).json({
                message : "Equipamento não encontrado"
            });   
        }
        res.json(equipament)
    } catch (erro){
        res.status(500).json({
            message: "Erro ao mudar status",
            erro
        })   
    }
}

export const equipamentController = {
    findAll,
    findById,
    create,
    updateByRFID,
    deleteByRFID,
    alertOutRange,
    changeStatus
}